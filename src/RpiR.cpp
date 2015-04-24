
#include <Rcpp.h>
#include <wiringPi.h>
#include <mcp3004.h>
#include <atomic>
#include <thread>
#include <chrono>

using namespace Rcpp;

#define BASE 100
#define SPI_CHAN 0

std::atomic<uint16_t> sharedValue(0);
uint16_t counter = 0;

void call_from_thread() {
  while (true){
    sharedValue.store(counter, std::memory_order_relaxed);
    counter++;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (counter > 10){
      return;
    }
  }
}

// [[Rcpp::export]]
void start_thread() {
  std::thread t1(call_from_thread);
  t1.detach();
}

// [[Rcpp::export]]
uint16_t read_val() {
  return sharedValue.load(std::memory_order_relaxed);
}

//' Initialize the Raspberry Pi for IO 
// [[Rcpp::export]]
void init(){
  wiringPiSetup(); 
  mcp3004Setup(BASE, SPI_CHAN); // 3004 and 3008 are the same 4/8 channels
}

//' Read an analog value fromt the Raspberry Pi
//' @param chan An integer vector of channel(s) for which we should
//'    read the analog value.
//' @return An integer vector describing the analog value on the 
//'    specified channel(s), ranging in value from 0-1024.
// [[Rcpp::export]]
NumericVector readAnalog(NumericVector chan) {
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
