#ifndef	__BED_STM32F4XX_H
#define		__BED_STM32F4XX_H
#include "stm32f4xx.h"                  // Device header

#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "stm32f4xx_hal_tim.h"
#include <stdbool.h>
#include <math.h>

//#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
//#include "task.h"


#define FULL 0
#define HALF 1
#define QUARTER 2
#define EIGHTH 3
#define SIXTEENTH 4



typedef struct
{
	uint32_t MsPin1;
	uint32_t MsPin2;
	uint32_t MsPin3;
	uint32_t DirectionPin;
	uint32_t StepPin;
	uint32_t EnablePin;
	uint32_t ResetPin;
	uint32_t SleepPin;
	
	// Pin GPIO Base
	GPIO_TypeDef* MsPinBase1;
	GPIO_TypeDef* MsPinBase2;
	GPIO_TypeDef* MsPinBase3;
	
	GPIO_TypeDef* DirectionPinBase;
	GPIO_TypeDef* StepPinBase;
	GPIO_TypeDef* EnablePinBase;
	GPIO_TypeDef* ResetPinBase;
	GPIO_TypeDef* SleepPinBase;
	
	struct motor
	{
		int StepsPerRev;
		
		double position;
		double target_position;
		double velocity;
		double acceleration;
	} motor;
	
} Big_Easy_Driver;




void bedPinOut(Big_Easy_Driver* bed, 
							 uint32_t Ms1, 
							 uint32_t Ms2,
							 uint32_t Ms3,
							 uint32_t En,
							 uint32_t dir,
						   uint32_t step);

void bedOptionalPinOut(Big_Easy_Driver* bed, uint32_t rst, uint32_t slp);

void bedPinBase(Big_Easy_Driver* bed, 
								GPIO_TypeDef* Ms1Base,
								GPIO_TypeDef* Ms2Base,
								GPIO_TypeDef* Ms3Base,
								GPIO_TypeDef* EnableBase,
								GPIO_TypeDef* DirectionBase,
								GPIO_TypeDef* StepBase);

void bedInit(Big_Easy_Driver* bed, GPIO_TypeDef* GPIOx, uint32_t pins);

void setDirection(Big_Easy_Driver* bed, GPIO_PinState dir, GPIO_TypeDef* directionPinBase);
void setStepResolution(Big_Easy_Driver* bed, int resolution, GPIO_TypeDef* StepPinBase);

void bedMoveXinc(Big_Easy_Driver* bed, double x_inc, double speed);
void bedMoveYinc(Big_Easy_Driver* bed, double y_inc, double speed);

void bedMoveXabs(Big_Easy_Driver* bed, double x_pos, double speed);
void bedMoveYabs(Big_Easy_Driver* bed, double y_pos, double speed);

void linearMoveXY(Big_Easy_Driver* bedX, Big_Easy_Driver* bedY, double x_pos, double y_pos, double speed);
void circularInterp(Big_Easy_Driver* bedX, Big_Easy_Driver* bedY, double x_pos, double y_pos, double rad, double speed);

void TIM2_init();
void TIM3_init();

#endif

