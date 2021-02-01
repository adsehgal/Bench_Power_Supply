<!-- ![](https://github.com/adsehgal/adsehgal/blob/master/LOGO.png) -->

![](LOGO_V1.1.png)

# *Bench Power Supply PCB*

This repo contains project files for a bench top power supply I designed for my home lab.

## *Requirements:*

* **Small** for crammed work spaces
* **Adjustable** and designed to **store values** on each power cycle
* **2 layer** board - V2.0 bumped to 4 layer
* **Digitally controlled** - no mechanical parts
* Clear **annunciators**
* Uses an external AC-DC converter
* At least a **2A max** load
* Cover most logic levels
* *Possibly have preset buttons*
  
## Built With

* [KiCad](https://kicad-pcb.org/) - Schematic capture and layout V1.0
* [VS Code](https://code.visualstudio.com/) - Used to develop firmware V1.0
* [Fusion 360](https://www.autodesk.com/education/edu-software/overview) - Used to design 3-D models for packages
* [Altium Designer](https://www.altium.com/) - Schematic capture and layout V2.0
* [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) - Used to develop firmware V2.0

### *Notes:*

* The PCB uses a 5/5mil DRC constraint
* The top of the PCB acts as the product face plate
* Non-standard footprint 3-D models have been provided in the STEP file format

## To Do
- [x] Connect OLED reset line to a GPIO</br>
- [x] Add reset pull up to MCU
- [x] ANDed interrupt for buttons
- [ ] Add config flash storage
- [ ] Bigger display?
- [x] Reduce inductor footprint
- [x] Add debouncing either in hardware or firmware, ideally both
- [x] Add a reinit function

## Authors

* **Aditya Sehgal** - *PCB Layout, Firmware, 3-D Modelling* - [Adsehgal](https://github.com/adsehgal)