
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
//' @param spi_channel If using an external ADC, which SPI channel is it running
//'   on?
//' @param pin_base The virtual GPIO pin number which we should use for the 
//'   fake pins we'll use for measuring analog values.
// [[Rcpp::export]]
void init(int spi_channel = 0, int pin_base = 100){
  base_pin = pin_base;
  wiringPiSetup(); 

  // TODO: make configurable
  mcp3004Setup(pin_base, spi_channel); // 3004 and 3008 are the same 4/8 channels

  // Init to -1 to show there are no active polls.
  for (int i = 0; i < MAX_CHANNELS; i++){
    next_write[i].store(-1, std::memory_order_relaxed);
  }
}

//' Read an analog value fromt the Raspberry Pi
//' @param chan An integer vector of channel(s) for which we should
//'    read the analog value.
//' @return An integer vector describing the current analog value on the 
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

// Internal scalar version of read_analog
int readAnalogScalar(int chan) {
  return analogRead(base_pin + chan);
}

// The function we'll run on new threads to poll for us.
void run_poll(int chan, int mms) {

  std::chrono::time_point<std::chrono::high_resolution_clock> last_run;

  while (next_write[chan].load(std::memory_order_relaxed) >= 0){
    last_run = std::chrono::high_resolution_clock::now();
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

    int lag_mms = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - last_run).count();
    // 75 micro seconds is empirically about the overhead for this untimed logic.
    std::this_thread::sleep_for(std::chrono::microseconds(mms - lag_mms - 75));
  }
}

//' Start a Poll
//' 
//' Creates a background C++ thread which will poll the given channel at the
//' specified interval, accumulating the values in a buffer until 
//' \code{\link{read_poll}} is called.
//' @param chan The channel number on which to poll.
//' @param ms The number of millisecons to wait in between each poll.
//' @param buffer_size The size of the buffer into which we should accumulate
//'   values.
// [[Rcpp::export]]
void start_poll(int chan, double ms = 1000, int buffer_size = 1024) {
  int mms = (int)(ms * 1000);

  if (next_write[chan].load(std::memory_order_relaxed) != -1){
    stop("Already have a poll running on this channel. Use stop_poll() to cancel it before starting a new poll.");
  }

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

//' Stop a Current Poll
//'
//' Stops any active poll on the specified channel, or does nothing if there
//' was no poll.
// [[Rcpp::export]]
void stop_poll(int chan){
  next_write[chan].store(-1, std::memory_order_relaxed);
  delete buffers[chan];
}

//' Reads From a Poll
//'
//' Read any unread values from an active poll. 
//'
//' If more values have accumulated than the buffer had room for, this function
//' will throw a warning and you will receive the most recent data.
//' @param chan The channel for which you want the unread values.
// [[Rcpp::export]]
NumericVector read_poll(int chan) {
  if (next_write[chan].load(std::memory_order_relaxed) == -1){
    stop("No poll active on the given channel.");
  }

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

