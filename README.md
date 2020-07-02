# HTU31D Generic C Driver
Generic C driver for the [HTU31D sensor](http://www.te.com/usa-en/product-CAT-HSC0007.html)

![htu31d](https://www.te.com/content/dam/te-com/catalog/part/CAT/HSC/000/CAT-HSC0007-t1.jpg/jcr:content/renditions/product-details.png)

The HTU31D sensor is a self-contained humidity and temperature sensor that is fully calibrated during manufacturing. Humidity measurements are now internaly compensated regarding temperature. The sensor can operate from 3.3V to 5.5V, has selectable resolution, internal diagnostics and checksum capability. The HTU31D has a low current stand-by mode for power-sensitive applications.

### Specifications
*	Measures relative humidity from 0% to 100%
*	Measures temperature from -40°C to 125°C
*	Humidity resolution selectable from 0.007% to 0.020%
*	Temperature resolution selectable from 0.012°C to 0.040°C
*	0.05 µA sleep current
*	450 µA operating current (during measurement only)
*	Typical accuracy +-2%RH and +-0.2°C

### Driver features
* HTU31D Reset request
* Aquisition resolution management
* Built-in heater management
* Read HTU31D Serial Number
* Read HTU31D diagnostics
* Read both or one of Temperature and Humidty measurement
* Calculate Dew Point


**NB:** This driver is intended to provide an implementation example of the sensor communication protocol, in order to be usable you have to implement a proper I2C layer for your target platform.
