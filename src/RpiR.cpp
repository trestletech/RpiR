
#include <Rcpp.h>
#include <wiringPi.h>
#include <mcp3004.h>

using namespace Rcpp;

#define BASE 100
#define SPI_CHAN 0

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
