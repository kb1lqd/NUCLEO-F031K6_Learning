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

# References

* GPIO Interrupt: https://www.youtube.com/watch?v=qd_tevhJ2eE
* Interrupt: https://www.2ofanerd.com/en/2020/06/17/easy-interrupt-handling-on-stm32-the-quick-tutorial/
