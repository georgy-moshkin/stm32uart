# stm32uart
Multi-port UART DMA library to use with STM32 HAL (Can be used on F103, F407, F446, H750, H7B0, etc.)
Can be modified (through defines) to work on almost any stm32 series.

Video tutorial: https://youtu.be/ZlsmW7s1N10

How-to (Using STM32CubeIDE):
1. Enable UARTS
2. Enable "global interrupt" for each UART
3. Enable DMA RX channel for each UART
4. Set DMA RX channel mode to Circular for each UART
5. Enable DMA TX channel for each UART
6. Generate code
7. Add UART_Init() for each uart (after HAL initialization functions, before main loop)
8. Use UART_ComSelect() to select uart
9. Use UART_Read() / UART_Write()

Hope this lib will solve all your UART problems.

Commented soure code is available in PDF version here: https://www.thundertronics.com/STM32_UART_Library.html
