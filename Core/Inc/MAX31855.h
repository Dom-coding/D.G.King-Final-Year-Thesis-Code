/*************************************************************************************
 Title	 :  MAX31855 header file
 Author  :  DG. King
 Software:  STM32CubeIDE
 Hardware:  STM32 NUCLEO F103RB
*************************************************************************************/
#ifndef MAX31855_H_
#define MAX31855_H_
#include "main.h"
// ------------------------- Defines -------------------------
extern uint8_t Error1;	   // Error Detection - 1-> No Connection / 2-> Short to GND / 4-> Short to VCC
extern uint8_t Error2;	   // Error Detection - 1-> No Connection / 2-> Short to GND / 4-> Short to VCC
extern uint8_t Error3;	   // Error Detection - 1-> No Connection / 2-> Short to GND / 4-> Short to VCC
extern uint8_t Error4;	   // Error Detection - 1-> No Connection / 2-> Short to GND / 4-> Short to VCC

#define SSPORT1 GPIOA       // GPIO Port of Chip Select1(Slave Select)
#define SSPIN1  GPIO_PIN_7  // GPIO PIN of Chip Select1(Slave Select)

#define SSPORT2 GPIOB       // GPIO Port of Chip Select2(Slave Select)
#define SSPIN2  GPIO_PIN_7  // GPIO PIN of Chip Select2(Slave Select)

#define SSPORT3 GPIOB       // GPIO Port of Chip Select3(Slave Select)
#define SSPIN3  GPIO_PIN_8  // GPIO PIN of Chip Select3(Slave Select)

#define SSPORT4 GPIOB       // GPIO Port of Chip Select4(Slave Select)
#define SSPIN4  GPIO_PIN_9  // GPIO PIN of Chip Select4(Slave Select)
// ------------------------- Functions  ----------------------
float Max31855_Read_Temp1(void);
float Max31855_Read_Temp2(void);
float Max31855_Read_Temp3(void);
float Max31855_Read_Temp4(void);
#endif
