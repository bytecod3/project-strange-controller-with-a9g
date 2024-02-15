## Project strange with A9G

### STM32 Timer configuration
The following screenshots show how to set the timer being used to achieve the ```time setting``` functionality:

1. Select timer 3

2. Set the parameters as shown in the below diagram (can be adjusted accordingly)

### Initializing GPS module 

### EEPROM memory
Add defines to memory location where to write time to. Change them according to the MCU you are using

```c
#define START_ADDRESS 
#define STOP_ADDRESS 
```

### Debugging
When deploying, set the ```DEBUG``` define in the main.c to 0

```c
#define DEBUG 1 // set to 0 to disable debugging
```

