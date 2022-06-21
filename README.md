# A home-made portable device based on Arduino Uno for pulsed magnetic resonance of NV centers in diamond

[![DOI](https://zenodo.org/badge/458715954.svg)](https://zenodo.org/badge/latestdoi/458715954)

# Introduction
This repository provides the code to operate a microntroller-based portable setup to analyze nitrogen-vacancy (NV) centers in diamond. The electron spin state of NV centers is initialized by optical pulses from a laser diode and manipulated by microwave (MW) pulses. The photoluminescence from NV centers is detected by a photodiode, converted in voltage which is read by the ADC of the microcontroller. The microcontroller generates the TTL pulses to create laser and microwave pulses.

By Giacomo Mariani

## How to cite


## Hardware requirements
The following devices are required to correctly operate the setup.

- MW sythesizer: ADF4351 module (Analog Devices)
- MW switch: Any which supports 5V TTL levels
- Laser driver: Op-amp + MOSFET 
- Switched integrator: IVC102 (Texas Instruments)
- The control of the RF signal requires the DDS AD9834 (Analog Devices)

## Code and software

- The repositories are available at https://doi.org/10.5281/zenodo.6657225
- The code was wirtten ans tested with the setup by using the open-source Arduino Software (IDE) ver. 1.8.13

## References
- The code to control the AD4351 module with the Arduino Uno microcontroller is readapted from a code written by Alain Fort F1CJN. The original code can be downloaded here: 
          <br/>- https://github.com/F1CJN/ARDUINO-ADF4351
          <br/>- http://f6kbf.free.fr/html/ADF4351%20and%20Arduino_Fr_Gb.htm

- The software to calculate the registers for the AD4351 initialization can be downloaded here
          <br/>https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/eval-adf4351.html#eb-relatedsoftware

- The code to control the AD9834 module with the Arduino Uno microcontroller is readapted from:
          <br/>- The AN-1070 APPLICATION NOTE (Analog Devices) titled "Programming the AD9833/AD9834" by Liam Riordan
          <br/>- The "AD9833 Waveform Module vwlowen.co.uk" code


