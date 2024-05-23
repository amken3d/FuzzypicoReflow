#!/usr/bin/python
import logging
import board
import digitalio
import adafruit_max31865


class MAX31865SPI(object):
    """Python driver for MAX31865 RTD PT100/PT1000 amplifier
    Requires:
    - adafruit's MAX31865 SPI device library
    """

    def __init__(self, cs_pin, rtd_nominal=None, ref_resistor=None, wires=None):
        spi = board.SPI()
        cs = digitalio.DigitalInOut(cs_pin)  # Chip select for the MAX31865 board.
        self.max31865 = adafruit_max31865.MAX31865(spi, cs, rtd_nominal,ref_resistor,wires )
        self.log = logging.getLogger(__name__)

    def get(self):
        '''Reads SPI bus and returns current value of thermocouple.'''
        try:
            temp = self.max31865.temperature
            self.log.debug("Temperature: {0:0.3f}C".format(temp))
            return temp
        except Exception as e:
            raise MAX31865SPIError(str(e))


class MAX31865SPIError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)
