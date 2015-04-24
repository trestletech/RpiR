
#include <Rcpp.h>
#include <wiringPi.h>
#include <mcp3004.h>

using namespace Rcpp;

#define BASE 100
#define SPI_CHAN 0

// [[Rcpp::export]]
void init(){
  wiringPiSetup(); 
  mcp3004Setup(BASE, SPI_CHAN); // 3004 and 3008 are the same 4/8 channels
}

// [[Rcpp::export]]
int readAnalog() {
  int x = analogRead (BASE + 0);
  return x;
}
