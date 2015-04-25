
## RpiR

An R package to interface with the RaspberryPi. Builds on the [Wiring Pi](wiringpi.com) library to support interaction with the Raspberry Pi's General Purpose I/O (GPIO) pins to read and write values to/from physical devices. You can, for instance, turn on an LED, read the state of a switch, or read the value of an analog sensor\* or even read in values from a microphone.

\* The Raspberry Pi doesn't have any built-in analog inputs, so analog sensors would need to be connected through an Analog-to-Digital Converter such as the [MCP3008](http://www.adafruit.com/products/856). ([Here's a guide](http://www.raspberrypi-spy.co.uk/2013/10/analogue-sensors-on-the-raspberry-pi-using-an-mcp3008/) on how to connect the Raspberry Pi to the chip).

### Analog Reading

Polling an analog input (as an integer between 0 and 1024) can be as easy as:

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
