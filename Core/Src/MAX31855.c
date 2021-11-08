/*************************************************************************************
 Title	 :	MAX31855
 Author  :	D.G King
 Software:  STM32CubeIDE
 Hardware:  STM32 Nucleo F103RB
*************************************************************************************/
#include"MAX31855.h"
SPI_HandleTypeDef hspi1;

// ------------------- Variables ----------------

uint8_t Error1=0;                                      // Thermocouple Connection acknowledge Flag for unit1
uint32_t sign1=0;									  // Sign bit for unit1
uint8_t DATARX1[4];                                    // Raw Data from MAX6675 for unit1

uint8_t Error2=0;                                      // Thermocouple Connection acknowledge Flag for unit2
uint32_t sign2=0;									  // Sign bit for unit2
uint8_t DATARX2[4];                                    // Raw Data from MAX6675 for unit2

uint8_t Error3=0;                                      // Thermocouple Connection acknowledge Flag for unit3
uint32_t sign3=0;									  // Sign bit for unit3
uint8_t DATARX3[4];                                    // Raw Data from MAX6675 for unit3

uint8_t Error4=0;                                      // Thermocouple Connection acknowledge Flag for unit4
uint32_t sign4=0;									  // Sign bit for unit4
uint8_t DATARX4[4];                                    // Raw Data from MAX6675 for unit4

// ------------------- Functions ----------------
float Max31855_Read_Temp1(void){
int Temp1=0;                                           // Temperature Variable
HAL_GPIO_WritePin(SSPORT1,SSPIN1,GPIO_PIN_RESET);       // Low State for SPI Communication
HAL_SPI_Receive(&hspi1,DATARX1,4,1000);                // DATA Transfer
HAL_GPIO_WritePin(SSPORT1,SSPIN1,GPIO_PIN_SET);         // High State for SPI Communication
Error1=DATARX1[3]&0x07;								  // Error Detection
sign1=(DATARX1[0]&(0x80))>>7;							  // Sign Bit calculation

if(DATARX1[3] & 0x07)								  // Returns Error Number
return(-1*(DATARX1[3] & 0x07));

else if(sign1==1){									  // Negative Temperature
Temp1 = (DATARX1[0] << 6) | (DATARX1[1] >> 2);
Temp1&=0b01111111111111;
Temp1^=0b01111111111111;
return((double)-Temp1/4);
	}

else												  // Positive Temperature
{
		Temp1 = (DATARX1[0] << 6) | (DATARX1[1] >> 2);
		return((double)Temp1 / 4);
}
}
//---------------------------------------------------------------function 2
float Max31855_Read_Temp2(void){
int Temp2=0;                                           // Temperature Variable
HAL_GPIO_WritePin(SSPORT2,SSPIN2,GPIO_PIN_RESET);       // Low State for SPI Communication
HAL_SPI_Receive(&hspi1,DATARX2,4,1000);                // DATA Transfer
HAL_GPIO_WritePin(SSPORT2,SSPIN2,GPIO_PIN_SET);         // High State for SPI Communication
Error2=DATARX2[3]&0x07;								  // Error Detection
sign2=(DATARX2[0]&(0x80))>>7;							  // Sign Bit calculation

if(DATARX2[3] & 0x07)								  // Returns Error Number
return(-1*(DATARX2[3] & 0x07));

else if(sign2==1){									  // Negative Temperature
Temp2 = (DATARX2[0] << 6) | (DATARX2[1] >> 2);
Temp2&=0b01111111111111;
Temp2^=0b01111111111111;
return((double)-Temp2/4);
	}

else												  // Positive Temperature
{
		Temp2 = (DATARX2[0] << 6) | (DATARX2[1] >> 2);
		return((double)Temp2 / 4);
}
}

//-------------------------------------------------------------function 3
float Max31855_Read_Temp3(void){
int Temp3=0;                                           // Temperature Variable
HAL_GPIO_WritePin(SSPORT3,SSPIN3,GPIO_PIN_RESET);       // Low State for SPI Communication
HAL_SPI_Receive(&hspi1,DATARX3,4,1000);                // DATA Transfer
HAL_GPIO_WritePin(SSPORT3,SSPIN3,GPIO_PIN_SET);         // High State for SPI Communication
Error3=DATARX3[3]&0x07;								  // Error Detection
sign3=(DATARX3[0]&(0x80))>>7;							  // Sign Bit calculation

if(DATARX3[3] & 0x07)								  // Returns Error Number
return(-1*(DATARX3[3] & 0x07));

else if(sign3==1){									  // Negative Temperature
Temp3 = (DATARX3[0] << 6) | (DATARX3[1] >> 2);
Temp3&=0b01111111111111;
Temp3^=0b01111111111111;
return((double)-Temp3/4);
	}

else												  // Positive Temperature
{
		Temp3 = (DATARX3[0] << 6) | (DATARX3[1] >> 2);
		return((double)Temp3 / 4);
}
}

//---------------------------------------------------------------function 4
float Max31855_Read_Temp4(void){
int Temp4=0;                                           // Temperature Variable
HAL_GPIO_WritePin(SSPORT4,SSPIN4,GPIO_PIN_RESET);       // Low State for SPI Communication
HAL_SPI_Receive(&hspi1,DATARX4,4,1000);                // DATA Transfer
HAL_GPIO_WritePin(SSPORT4,SSPIN4,GPIO_PIN_SET);         // High State for SPI Communication
Error4=DATARX4[3]&0x07;								  // Error Detection
sign4=(DATARX4[0]&(0x80))>>7;							  // Sign Bit calculation

if(DATARX4[3] & 0x07)								  // Returns Error Number
return(-1*(DATARX4[3] & 0x07));

else if(sign4==1){									  // Negative Temperature
Temp4 = (DATARX4[0] << 6) | (DATARX4[1] >> 2);
Temp4&=0b01111111111111;
Temp4^=0b01111111111111;
return((double)-Temp4/4);
	}

else												  // Positive Temperature
{
		Temp4 = (DATARX4[0] << 6) | (DATARX4[1] >> 2);
		return((double)Temp4 / 4);
}
}
