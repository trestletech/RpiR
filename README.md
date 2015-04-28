
## RpiR

An R package to interface with the RaspberryPi. Builds on the [Wiring Pi](wiringpi.com) library to support interaction with the Raspberry Pi's General Purpose I/O (GPIO) pins to read and write values to/from physical devices. You can, for instance, turn on an LED, read the state of a switch, or read the value of an analog sensor\* or even read in values from a microphone.

> \* The Raspberry Pi doesn't have any built-in analog inputs, so analog sensors would need to be connected through an Analog-to-Digital Converter such as the [MCP3008](http://www.adafruit.com/products/856). ([Here's a guide](http://www.raspberrypi-spy.co.uk/2013/10/analogue-sensors-on-the-raspberry-pi-using-an-mcp3008/) on how to connect the Raspberry Pi to the chip).

### Setup & Installation

You'll need to download and install Wiring Pi on your Raspberry Pi first. Instructions to do so are [available here](http://wiringpi.com/download-and-install/).

You can test to confirm that your hardware is setup and wired as expected using the [GPIO utility](http://wiringpi.com/the-gpio-utility/) that comes with Wiring Pi, for instance:

```bash
$ gpio -x mcp3004:100:0 aread 100
515
```

Once you have Wiring Pi and your hardware configured properly, you can move on to install RpiR.

Since RpiR is not available on CRAN, you'll want to install from GitHub. The easiest way to do that is using the `devtools` package.

```r
> install.packages("devtools")
> library(devtools)
> install_github("trestletech/RpiR")
> library(RpiR)
```

Also, Wiring Pi requires sudo/root privileges for many operations. If you're not concerned with security, the easiest solution is just to run R with sudo privileges.

### Digital Reading & Writing

```r
> library(RpiR)
> RpiR::init()
> RpiR::pin_mode(1, "in")
> RpiR::read_digital(1)
[1] TRUE
>
>
> RpiR::pin_mode(2, "out")
> RpiR::write_digital(2, TRUE)  # To write a 1/HIGH to pin 2
> RpiR::write_digital(2, FALSE) # To write a 0/LOW to pin 2
```

### Analog Reading & Writing

Polling an analog input can be as easy as:

```r
> library(RpiR)
> RpiR::init()
> RpiR::read_analog(0)
[1] 752
```

However, for very fast polling or polling that requires tighter bounds on the interval between polls, RpiR offers a polling function that will spawn a background thread to poll the input for you at regular intervals. 

```r
> RpiR::start_poll(0, ms = 100) # Poll on analog channel 0 every 100 milliseconds
> RpiR::read_poll(0)
 [1] 618 606 597 584 572 567 563 561 552 551 551 549 547
> RpiR::stop_poll(0)
```

Preliminary testing on a Raspberry Pi 2 Model B shows a maximum analog polling frequency approaching 10kHz using a MCP3008 ADC.

Likewise, `write_analog` or `write_pwm` can be used to write an analog signal via ADC or PWM, respectively.

### Controlling Pins

`pin_mode` can be used to set the purpose of a pin to either input, digital output, PWM output, or a GPIO clock.

`pin_control` is used to set the internal resistor for a pin to either off, up, or down.
