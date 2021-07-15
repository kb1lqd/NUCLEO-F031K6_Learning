# NUCLEO-F031K6_Learning

# Work Log

## 7/12/21

* Initial project setup
* Wired PMOD rotary button to the PCBA
* Learned to setup basic GPIO interrupts
* Got a basic test of the GPIO interrupt detection for CW and CCW working

### For Next Time

* Implement a proper detection of CW and CCW detection that is reliable and not based on poorly defined detection code inside the interrupt.
  * I may want to pursue a simple state machine here.


## 7/15/21

* Implemented basic interrupt GPIO CW/CCW detection
  * It is not debounced and is glitchy
* Implemented UART text output
* Implemented basic us counter with TIM16
* Transmits current count every 20us via TIMER

### For Next Time

* Implement basic switch debounce using the timer to clean up rotary detection
* I could implement rotary detection via the internal hardware support but this requires specific pins
* I could implement rotary control of a PWM'd LED for fun
* wire up LCD

# References

* GPIO Interrupt: https://www.youtube.com/watch?v=qd_tevhJ2eE
* Interrupt: https://www.2ofanerd.com/en/2020/06/17/easy-interrupt-handling-on-stm32-the-quick-tutorial/
* TIMER Encoder Mode: https://deepbluembedded.com/stm32-timer-encoder-mode-stm32-rotary-encoder-interfacing/
* SPLC780D LCD CHAR Library: https://github.com/coocox/cox/blob/master/CoX/Driver/LCD_Character/SPLC780D/Character_LCD_SPLC780D_Driver/lib/splc780d.c
* Debounce Timer: http://www.emcu.it/STM32/STM32Discovery-Debounce/STM32Discovery-InputWithDebounce_Output_UART_SPI_SysTick.html
* Timer Interrupt: https://www.youtube.com/watch?v=VfbW6nfG4kw
