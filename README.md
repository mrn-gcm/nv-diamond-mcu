# DiamondNVcenters_microcontroller
This repository provides the code to operate a microntroller-based portable setup to analyze nitrogen-vacancy centers in diamond.

## Hardware requirements
The following devices are required to correctly operate the setup.

- MW sythesizer: ADF4351 module (Analog Devices)
- Laser driver: Op-amp + MOSFET 
- Switched integrator: IVC102 (Texas Instruments)

The control of the RF signal required the DDS AD9834 (Analog Devices)


## References
- The code to control the AD4351 module with the Arduio Uno microcontroller is readapted from a code written by Alain Fort F1CJN. The original code can be downloaded here: 
https://github.com/F1CJN/ARDUINO-ADF4351
http://f6kbf.free.fr/html/ADF4351%20and%20Arduino_Fr_Gb.htm


