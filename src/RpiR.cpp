
#include <Rcpp.h>
#include <wiringPi.h>
#include <mcp3004.h>
#include <atomic>
#include <thread>
#include <chrono>

using namespace Rcpp;

std::atomic<uint16_t> sharedValue(0);
std::atomic<bool> running(false);
uint16_t counter = 0;

void run_poll(int chan, int mms) {
  while (running.load(std::memory_order_relaxed)){
    counter++;
    sharedValue.store(counter, std::memory_order_relaxed);
    std::this_thread::sleep_for(std::chrono::microseconds(mms));
  }
}

// [[Rcpp::export]]
void start_poll(int chan, double ms) {
  int mms = (int)(ms * 1000);
  running.store(true, std::memory_order_relaxed);
  std::thread t1(run_poll, chan, mms);
  t1.detach();
}

// [[Rcpp::export]]
void stop_poll(int chan){
  running.store(false, std::memory_order_relaxed);
}

// [[Rcpp::export]]
uint16_t read_val() {
  return sharedValue.load(std::memory_order_relaxed);
}

//' Initialize the Raspberry Pi for IO 
// [[Rcpp::export]]
void init(int spi_channel = 0, int pin_base = 100){
  wiringPiSetup(); 
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
    out[i] = analogRead (BASE + chan[i]);    
  }
  return out;
}

// [[Rcpp::export]]
int readAnalogScalar(int chan) {
  return analogRead(BASE + chan);
}
