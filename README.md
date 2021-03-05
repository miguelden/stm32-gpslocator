# stm32-gpslocator

This project is the implementation of usage of the GPSlocator library on a STM32F4-Discovery board.

Includes:
- UART configuration read the messages received from GPS module, using DMA circular buffer to received data.
- DMA half-transfer and transfer complete interrupt handler, and also USART idle line interrupt handler
- GPSlocator app_step execution
- Circular timer to manage three LED blinking with the following information:
    - System execution
    - GPS Fix active
    - Device is on range of target


## Download the project

To clone the repository:

```
$ git clone https://github.com/miguelden/stm32-gpslocator.git --recursive
```


## Build the application

The STM32CubeIDE environment was used to compile this application and updload the image to the STM32F4-Discovery.
