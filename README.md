# libEPOS_STM32

- An MAXON EPOS driver for STM32；
- Support EPOS and EPOS 2 (not tested in other versions of EPOS)；
- Use CAN communication；
- Use STM32 HAL library;

# Usage
1. Init CAN bus
2. Open EPOS using the CAN device and the EPOS CAN ID
3. Call EPOS functions