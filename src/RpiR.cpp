
#include <Rcpp.h>
#include <wiringPi.h>
#include <mcp3004.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <unistd.h>

using namespace Rcpp;

#define MAX_CHANNELS 16

int *buffers[MAX_CHANNELS];
std::atomic<int> next_write[MAX_CHANNELS] = {};
std::atomic<int> last_read[MAX_CHANNELS] = {};
int sizes[MAX_CHANNELS];

int base_pin;

std::string current_mode;

//' Initialize the Raspberry Pi for IO 
//' @param setup_type Which mode wiringPi should be initialized in. Either
//'   \code{wpi}, \code{gpio}, \code{phys}, or \code{sys}. See
//'   \url{http://wiringpi.com/reference/setup/} for definitions of each.
//' @param extension The base name of the extension to add, e.g. \code{mcp3004}.
//' @param spi_i2c If using an external ADC, which SPI channel is it running
//'   on, or the address to use for i2c. Look at the documentation for the
//'   wiringPi extension for details for your particular ADC.
//' @param pin_base The virtual GPIO pin number which we should use for the 
//'   fake pins we'll use for measuring analog values.
// [[Rcpp::export]]
void init(std::string setup_type="wpi", CharacterVector extension = {"mcp23008","mcp23016","mcp23017","mcp23s08","mcp23s17","sr595","pcf8574","pcf8591","max31855","mcp3002","mcp3004","max5322","mcp4802","sn3218","mcp3422"}, int spi_i2c = 0, int pin_base = 100){
  base_pin = pin_base;

  // I hate to implement this here, but we need to avoid the exit() call from wiringPi.
  // The most common route that would cause a fatal exit is not being root, so we'll
  // just check for that ourselves in each setup case that requires root.
  if (setup_type != "sys" && geteuid() != 0){
    stop("To run with this setup_type, you must be root.");
  }

  if (setup_type == "wpi"){
    wiringPiSetup();
  } else if (setup_type == "gpio") {
    wiringPiSetupGpio();
  } else if (setup_type == "phys") {
    wiringPiSetupPhys();
  } else if (setup_type == "sys") {
    wiringPiSetupSys();
  } else {
    char error_buffer[64];
    snprintf(error_buffer, 64, "Unrecognized setup_type: %s", setup_type.c_str());
    stop(error_buffer);
  }
  current_mode = setup_type;

  if (extension[0] == "mcp23008"){
    mcp23008Setup(pinBase, spi_i2c_i2c);
  } else if (extension[0] == "mcp23016"){
    mcp23016Setup(pinBase, spi_i2c_i2c);
  } else if (extension[0] == "mcp23017"){
    mcp23017Setup(pinBase, spi_i2c_i2c);
  } else if (extension[0] == "mcp23s08"){
		stop("The mcp23s08 is not yet supported.");
  } else if (extension[0] == "mcp23s17"){
		stop("The mcp23s17 is not yet supported.");
  } else if (extension[0] == "sr595"){
		stop("The sr595 is not yet supported.");
  } else if (extension[0] == "pcf8574"){
    pcf8574Setup(pinBase, spi_i2c_i2c);
  } else if (extension[0] == "pcf8591"){
    pcf8591Setup(pinBase, spi_i2c_i2c);
  } else if (extension[0] == "max31855"){
    max31855Setup(pinBase, spi_i2c);
  } else if (extension[0] == "mcp3002"){
    mcp3002Setup(pinBase, spi_i2c);
  } else if (extension[0] == "mcp3004"){
    mcp3004Setup(pinBase, spi_i2c);
  } else if (extension[0] == "max5322"){
    max5322Setup(pinBase, spi_i2c);
  } else if (extension[0] == "mcp4802"){
    mcp4802Setup(pinBase, spi_i2c);
  } else if (extension[0] == "sn3218"){
		sn3218Setup(pinBase);
  } else if (extension[0] == "mcp3422"){
		mcp3422Setup(pinBase);
	} else if (extension[0] == ""){
		// No-op
	} else {
		stop("Unrecognized extension[0].");
	}


  // Init to -1 to show there are no active polls.
  for (int i = 0; i < MAX_CHANNELS; i++){
    next_write[i].store(-1, std::memory_order_relaxed);
  }
}

//' Read an analog value fromt the Raspberry Pi
//' @param chan An integer vector of channel(s) for which we should
//'    read the analog value.
//' @return An integer vector describing the current analog value on the 
//'    specified channel(s).
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

//' Set the mode of a GPIO pin
// [[Rcpp::export]]
void pin_mode(int pin, CharacterVector mode = CharacterVector::create("in", "out", "pwm", "clock")){
  int mode_i;
  
	if (current_mode == "sys"){
		Function warning("warning");
		warning("Setting the pin mode has no effect when in sys mode.");
	}

  if (mode[0] == "in"){
    mode_i = INPUT;
  } else if (mode[0] == "out"){
		mode_i = OUTPUT;
  } else if (mode[0] == "pwm"){
		mode_i = PWM_OUTPUT;
  } else if (mode[0] == "clock"){
		mode_i = GPIO_CLOCK;
	}

  pinMode(pin, mode_i);
}

//' Pull a Pin Up or Down
// [[Rcpp::export]]
void pin_control(int pin, CharacterVector mode = CharacterVector::create("off", "down", "up")){
  int mode_i;
  
  if (mode[0] == "off"){
    mode_i = PUD_OFF;
  } else if (mode[0] == "down"){
		mode_i = PUD_DOWN;
  } else if (mode[0] == "up"){
		mode_i = PUD_UP;
	} else {
    char error_buffer[64];
    snprintf(error_buffer, 64, "Unrecognized mode: %s", as<std::string>(mode[0]).c_str());
    stop(error_buffer);
  }

  pullUpDnControl(pin, mode_i);
}

//' Digital Write to a Pin
// [[Rcpp::export]]
void write_digital(int pin, bool signal){
  int val = LOW;

  if (signal){
		val = HIGH;
  } 

  digitalWrite(pin, signal);
}

//' Digital Read from a Pin
// [[Rcpp::export]]
bool read_digital(int pin){
  int val = digitalRead(pin);
  if (val == HIGH){
		return true;
	} else {
		return false;
	}
}

//' Write PWM to a signal
// [[Rcpp::export]]
void write_pwm(int pin, int value){
  if (value < 0 || value > 1024){
		stop("Value must be between 0 and 1024.");
	}

	pwmWrite(pin, value);
}


//' Write an Analog Signal
// [[Rcpp::export]]
void write_analog(int pin, int value){
	analogWrite(pin, value);
}
