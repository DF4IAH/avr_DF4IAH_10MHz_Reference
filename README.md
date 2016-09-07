# avr_DF4IAH_10MHz_Reference

## GPS disciplined 10 MHz reference oscillator based on a (VC)TCXO
The project name for this device is __10 MHz-Ref.-Osc. V2.x__

That board is attached at the USB bus and receives power and does communication over it.
An optional LCD module can be connected to the I²C compatible bus for a stand-alone operation.

The author is Ulrich Habel (DF4IAH) with the idea having a low-cost and low-power device for a module to generate a 10 MHz signal at 3.3V TTL, that is adjusted to the GPS/Glonass satellite clock system. By this the local clock is kept in its boundaries to satisfy the long-term stability of the GPS/Glonass system. The negative aspect on it (I want to inform, also) with the use of a low power consuming VC-TCXO is, that the short-term variance is much higher than the variance of the devices using a temperature controlled oscillator like a rubidium lightwave controlled one.

### Contents of this repository
- __Firmware__: MPU software managing the GPS receiver, the pull voltage for the VC-TCXO, the USB communication and an optional LCD for data presentation

### Additional information
The idea behind that project was to craft a local oscillator being in phase of the GPS clock system.
The device shall have a frequency and phase locked loop.
When the local frequency is trimmed to be equal to the reference GPS clock the phase locked loop gets in place of the frequency control.
Whenever the phase gets out of the secured boundaries, the system switches back to the frequency correction mode.
Due to its nature the device has to keep itself earnestly in the secured boundaries.
Only then the high quality stability of the GPS clock system can be delivered to the user.
For this reason when the local clock gets more out of balance and to one of its boundaries, a phase correction impulse is added to the local system to find the balance, again.
When still running a longer time this pulses are fine grained and the performance is very well for the user.
On the other hand when the GPS signals are poor, the almanac data old or the temperature variance at the VC-TCXO increasing, that local error-signal has to be corrected sometimes with sharp corrections.
So be warned to keep the device running in an air-locked container and a good GPS antenna position is a __must__ to get good results.

There are three different kinds of GPS units used by this project.
Each of them are utilizing the __MediaTek MT3333__ chipset, but each of the manufacturer is running its own configuration on it.
By this, some minor differences occur.
All of this GPS devices are able to track on GPS (western satellites) and Glonass/BeiDou (eastern satellites) signals.
Later at the works the author found out that the parallel processing of GPS and Glonass can result to interrupted functionality.
With NMEA programming the GPS device receives the command to ignore the Glonass signals but not all of them do honor that and start up tracking of all received signals.
The only work-around for this is to use a more selective GPS-antenna that filters out the Glonass signals away from the single GPS frequency.

The author has his reference clocks running for month without any problems, the firmware is stable - __use on your own risk, only!__

If you find this project nice you are welcomed to clone from it and run your optimizations or having a start point for a fresh project.

Any new modifications may result to harm this behavior.
Make your own decisions, be warned.

### Credits
I like to thank these following programmers and groups:

- __VUSB__ This code helped me to have USB communication on an ATmega, that does not support USB communication on its own.
All is done by interrupt software, many parts are assembly language based code.
Due to this nature all code in the bootloader as well as the firmware are very sensitive to changes on the code.
Only a very short interrupt time is allowed for the firmware avoiding the USB communication to fade out.
Find more about it: https://www.obdev.at/products/vusb/index.html

- __Wouter van Ooijen__ That person has reserved a valid USB address to be used by the community.
I like to thank him, that I am a part of this open source development community.

- __USBasp__ The base idea of the circuit is a reflection of this AVR programmer device.
The clou of it, it can communicate at the USB bus by itself: http://www.fischl.de/usbasp/

- __avrdude__ The bootloader code of avrdude gave ideas how to set-up a USB communication server.
The avrdude software can be found there: http://www.nongnu.org/avrdude/

- __HF-Messzwerg__ When the author got in contact with the team of this device, he started to learn about SMD soldering and USB communication in relation with an ATmega.
Communication is done with the help of an USB/serial converter.
More about that project can be found there: http://ukw-tagung.org/2016/preistraeger-2/

I wish you a nice experience with this product.
Fell free and enjoy it.

Cheers

Ulrich Habel  (DF4IAH)
