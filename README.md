# rpi-clicker
NRF24L01+ library for the RaspberryPi (tested on a Rev1), configured such that it may easily interface with TurningPoint classroom feedback systems.

### Configuration
When connecting the nrf24l01+, the only pins necessary are 3.3v, GND, and the 4 SPI pins (MOSI, MISO, CLK, CS). For locating these on the Pi refer to these pinouts for revision 1 and 2, courtesy of jeffskinnerbox and pinballsp.

http://www.megaleecher.net/sites/default/files/images/rpi-pin-style-1.jpg
![Alt Text](http://www.megaleecher.net/sites/default/files/images/rpi-pin-style-1.jpg "RaspberryPi v1 Pinout")

http://www.megaleecher.net/Raspberry_Pi_GPIO_Pinout_Helper
![Alt Text](http://www.megaleecher.net/sites/default/files/images/raspberry-pi-rev2-gpio-pinout.jpg "RaspberryPi v2 Pinout")

Chip select 0 is used. The interrupt pin on the nrf chip is a n/c.

### Dependencies
Uses Stefan Engelke's NRF24L01+ register definitions: ``` nRF24L01.h ```
