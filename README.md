## Project strange with A9G

### STM32 HAL header file inclusion
Before building, check all header files in the ```src``` and ```core``` where ```stm32f4xx_hal.h``` is being included 
and change it to ```stm32f1xx_hal.h```.

### STM32 Timer configuration

1. Enable RCC external crystal clock source
2. Set

The following parameters are used for the timer.
```c
Clock frequency: 72 MHz
Prescaler: 7200
Timer clock frequency: 10kHz
Timer period register(ARR): 10000
Timer period: 1ms
```

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

Check flash memory addresses in the ```FLASH_PAGE_F1.h```

### Debugging
When deploying, set the ```DEBUG``` define in the main.c to 0



```c
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

...

#define DEBUG 1 // set to 0 to disable debugging


/* USER CODE END PD */
```

