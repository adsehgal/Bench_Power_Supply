<!-- ![](https://github.com/adsehgal/adsehgal/blob/master/LOGO.png) -->

![](LOGO_V1.1.png)

# _Bench Power Supply PCB_

This repo contains project files for a bench top power supply I designed for my home lab.

## _Requirements:_

- **Small** for crammed work spaces
- **Adjustable** and designed to **store values** on each power cycle
- **2 layer** board - V2.0 bumped to 4 layer
- **Digitally controlled** - no mechanical parts
- Clear **annunciators**
- Uses an external AC-DC converter
- At least a **2A max** load
- Cover most logic levels
- _Possibly have preset buttons_

## Built With:

#### V1.0:

- [KiCad](https://kicad-pcb.org/) - Schematic capture and layout V1.0
- [VS Code | PlatformIO](https://code.visualstudio.com/) - Used to develop firmware V1.0
- [Fusion 360](https://www.autodesk.com/education/edu-software/overview) - Used to design 3-D models for packages

#### V2.0:

- [Altium Designer](https://www.altium.com/) - Schematic capture and layout V2.0
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) - Used to develop firmware V2.0

## Notes:

- The PCB uses a 5/5mil DRC constraint
- The top of the PCB acts as the product face plate

## Firmware Changes:

| Change Type          | V 1.0     | V2.0       |
| -------------------- | --------- | ---------- |
| MCU Architecture     | 8-bit AVR | 32-bit ARM |
| User Input Handling  | Polling   | Interrupts |
| Analog Readings      | Polling   | DMA        |
| Serial Communication | None      | Added      |
| Operating System     | None      | FREERTOS   |
|                      |           |            |

## Hardware Changes:

| Change Type       | V 1.0             | V2.0              |
| ----------------- | ----------------- | ----------------- |
| MCU Selection     | ATMega 328p       | STM32F401RB       |
| Digi Pot          | MCP4023 - Up/Down | MCP4018 - I2C     |
| Inductor Seletion | 4.7µH - 7A        | 4.7µH - 4A        |
| PCB Stackup       | 2-layer           | 4-layer           |
| User Input Buffer | None              | Schmitt triggered |
| Analog Buffers    | LM358 - Not RRIO  | TLV271 - RRIO     |
| MCU VDD           | Not always on     | Always On         |
| OLED Reset        | None              | Added             |
|                   |                   |                   |

## Authors

- **Aditya Sehgal** - _PCB Layout, Firmware, 3-D Modelling_ - [Adsehgal](https://github.com/adsehgal)
