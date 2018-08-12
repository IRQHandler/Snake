# Snake

A microcontroller project that replicates the classic game "Snake" on a purpose-built LED screen and 8-bit controller. 

The hardware is controlled by an STM32F051 Discovery board (ARM Cortex-M0 architecture). The game logic and hardware interaction is programmed in C, with development and debugging done using the IAR EWARM IDE. A linker .map file is included, showing the memory layout of the program, a stack usage analysis, and the final memory usage. Source code for the MCU vendor #include files used in the project can be found at: https://www.st.com/stm32f0discovery
