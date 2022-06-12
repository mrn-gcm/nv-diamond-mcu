# DiamondNVcenters_microcontroller
This repository provides the code to operate a microntroller-based portable setup to analyze nitrogen-vacancy centers in diamond.

The following devices are required to correctly operate the setup.

- MW sythesizer: ADF4351 module (Analog Devices)
- Laser driver: Op-amp + MOSFET 
- Switched integrator: IVC102 (Texas Instruments)

The control of the RF signal required the DDS AD9834 (Analog Devices)



The code to control the AD4351 module is readapted from a code written by Alain Fort. The original code can be downloaded here: http://f6kbf.free.fr/html/ADF4351%20and%20Arduino_Fr_Gb.htm

