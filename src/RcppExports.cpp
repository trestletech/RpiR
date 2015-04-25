// This file was generated by Rcpp::compileAttributes
// Generator token: 10BE3573-1514-4C36-9D1C-5A225CD40393

#include <Rcpp.h>

using namespace Rcpp;

// init
void init(int spi_channel, int pin_base);
RcppExport SEXP RpiR_init(SEXP spi_channelSEXP, SEXP pin_baseSEXP) {
BEGIN_RCPP
    Rcpp::RNGScope __rngScope;
    Rcpp::traits::input_parameter< int >::type spi_channel(spi_channelSEXP);
    Rcpp::traits::input_parameter< int >::type pin_base(pin_baseSEXP);
    init(spi_channel, pin_base);
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
// start_poll
void start_poll(int chan, double ms, int buffer_size);
RcppExport SEXP RpiR_start_poll(SEXP chanSEXP, SEXP msSEXP, SEXP buffer_sizeSEXP) {
BEGIN_RCPP
    Rcpp::RNGScope __rngScope;
    Rcpp::traits::input_parameter< int >::type chan(chanSEXP);
    Rcpp::traits::input_parameter< double >::type ms(msSEXP);
    Rcpp::traits::input_parameter< int >::type buffer_size(buffer_sizeSEXP);
    start_poll(chan, ms, buffer_size);
    return R_NilValue;
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
// read_poll
NumericVector read_poll(int chan);
RcppExport SEXP RpiR_read_poll(SEXP chanSEXP) {
BEGIN_RCPP
    Rcpp::RObject __result;
    Rcpp::RNGScope __rngScope;
    Rcpp::traits::input_parameter< int >::type chan(chanSEXP);
    __result = Rcpp::wrap(read_poll(chan));
    return __result;
END_RCPP
}
