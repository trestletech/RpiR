
#include <Rcpp.h>
#include <wiringPi.h>
#include <mcp3004.h>
#include <atomic>
#include <thread>
#include <chrono>

using namespace Rcpp;

#define MAX_CHANNELS 16

int *buffers[MAX_CHANNELS];
std::atomic<int> next_write[MAX_CHANNELS] = {};
std::atomic<int> last_read[MAX_CHANNELS] = {};
int sizes[MAX_CHANNELS];

int base_pin;

//' Initialize the Raspberry Pi for IO 
// [[Rcpp::export]]
void init(int spi_channel = 0, int pin_base = 100){
  base_pin = pin_base;
  wiringPiSetup(); 

  // TODO: make configurable
  mcp3004Setup(pin_base, spi_channel); // 3004 and 3008 are the same 4/8 channels
}

//' Read an analog value fromt the Raspberry Pi
//' @param chan An integer vector of channel(s) for which we should
//'    read the analog value.
//' @return An integer vector describing the analog value on the 
//'    specified channel(s), ranging in value from 0-1024.
// [[Rcpp::export]]
NumericVector read_analog(NumericVector chan) {
  int n = chan.size();
  NumericVector out(n);

  for (int i = 0; i < n; i++){
    out[i] = analogRead (base_pin + chan[i]);    
  }
  return out;
}

int readAnalogScalar(int chan) {
  return analogRead(base_pin + chan);
}

void run_poll(int chan, int mms) {
  while (next_write[chan].load(std::memory_order_relaxed) >= 0){
    int ptr = next_write[chan].load(std::memory_order_relaxed);
    buffers[chan][ptr] = readAnalogScalar(chan);

    ptr++;
    if (ptr == last_read[chan] + 1) {
      // Overflow!
      // We're now pointing at the cell that would be included in the next read.
      last_read[chan].store(-1, std::memory_order_relaxed);
    }
    if (ptr >= sizes[chan]) {
      ptr = 0;
    }
    next_write[chan].store(ptr, std::memory_order_relaxed);

    std::this_thread::sleep_for(std::chrono::microseconds(mms)); //TODO: minus loop execution time
  }
}

// [[Rcpp::export]]
void start_poll(int chan, double ms = 1000, int buffer_size = 1024) {
  int mms = (int)(ms * 1000);

  if (chan > MAX_CHANNELS || chan < 0){
    forward_exception_to_r(std::range_error("Don't support the given channel number."));
  }

  // Initialize buffer
  buffers[chan] = new int[buffer_size + 1];
  sizes[chan] = buffer_size + 1;
  next_write[chan].store(0, std::memory_order_relaxed);
  last_read[chan].store(sizes[chan] - 1, std::memory_order_relaxed);

  std::thread t1(run_poll, chan, mms);
  t1.detach();
}

// [[Rcpp::export]]
void stop_poll(int chan){
  // TODO: delete array
}

// [[Rcpp::export]]
NumericVector read_val(int chan) {
  int start = last_read[chan].load(std::memory_order_relaxed) + 1;
  int stop = next_write[chan].load(std::memory_order_relaxed) - 1;

  if (start == 0) {
    // Means last_read was -1, i.e. overflowed
    Function warning("warning");
    warning("Values overflowed the buffer. Consider reading more often or increasing the size of the buffer.");
    start = stop + 2;
    if (start >= sizes[chan]){
      start = 0;
    }
  }

  if (start == stop + 1){
    // No new data to read.
    return NumericVector(0);
  }

  if (stop < 0){
    stop = sizes[chan]-1;
  }

  // Update the last unread marker
  last_read[chan].store(stop, std::memory_order_relaxed);

  // calculate the length
  int length;
  if (stop > start) {
    length = stop - start + 1;
  } else if (start > stop){
    length = sizes[chan] - start + stop + 1;
  } else {
    // ==
    length = 1;    
  }

  NumericVector out(length);
  for (int i = 0; i < length; i++){
    int ptr = start + i;
    if (ptr >= sizes[chan]){
      ptr -= sizes[chan];
    }
    out[i] = buffers[chan][ptr];
  }

  return out;
}

