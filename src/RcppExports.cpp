// This file was generated by Rcpp::compileAttributes
// Generator token: 10BE3573-1514-4C36-9D1C-5A225CD40393

#include <Rcpp.h>

using namespace Rcpp;

// start_poll
int start_poll(int chan, double ms);
RcppExport SEXP RpiR_start_poll(SEXP chanSEXP, SEXP msSEXP) {
BEGIN_RCPP
    Rcpp::RObject __result;
    Rcpp::RNGScope __rngScope;
    Rcpp::traits::input_parameter< int >::type chan(chanSEXP);
    Rcpp::traits::input_parameter< double >::type ms(msSEXP);
    __result = Rcpp::wrap(start_poll(chan, ms));
    return __result;
END_RCPP
}
// stop_poll
void stop_poll(int chan);
RcppExport SEXP RpiR_stop_poll(SEXP chanSEXP) {
BEGIN_RCPP
    Rcpp::RNGScope __rngScope;
    Rcpp::traits::input_parameter< int >::type chan(chanSEXP);
    stop_poll(chan);
    return R_NilValue;
END_RCPP
}
// read_val
uint16_t read_val();
RcppExport SEXP RpiR_read_val() {
BEGIN_RCPP
    Rcpp::RObject __result;
    Rcpp::RNGScope __rngScope;
    __result = Rcpp::wrap(read_val());
    return __result;
END_RCPP
}
// init
void init();
RcppExport SEXP RpiR_init() {
BEGIN_RCPP
    Rcpp::RNGScope __rngScope;
    init();
    return R_NilValue;
END_RCPP
}
// read_analog
NumericVector read_analog(NumericVector chan);
RcppExport SEXP RpiR_read_analog(SEXP chanSEXP) {
BEGIN_RCPP
    Rcpp::RObject __result;
    Rcpp::RNGScope __rngScope;
    Rcpp::traits::input_parameter< NumericVector >::type chan(chanSEXP);
    __result = Rcpp::wrap(read_analog(chan));
    return __result;
END_RCPP
}
// readAnalogScalar
int readAnalogScalar(int chan);
RcppExport SEXP RpiR_readAnalogScalar(SEXP chanSEXP) {
BEGIN_RCPP
    Rcpp::RObject __result;
    Rcpp::RNGScope __rngScope;
    Rcpp::traits::input_parameter< int >::type chan(chanSEXP);
    __result = Rcpp::wrap(readAnalogScalar(chan));
    return __result;
END_RCPP
}
