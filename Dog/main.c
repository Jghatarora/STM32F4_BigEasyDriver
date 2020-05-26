#include "stm32f4xx.h"                  // Device header
#include "bed_stm32F4xx.h"
// Define pins to be used for Big Easy Driver

#define MS1_PINx											GPIO_PIN_12 // pin 0 to be used with PB0 for Green LED
#define MS2_PINx											GPIO_PIN_13 // pin 7 for PB7 for Blue LED
#define MS3_PINx											GPIO_PIN_15 // pin 14 for pb14 connected to Red LED
#define ENABLE_PINx 									GPIO_PIN_8 // use pin 3 for enable
#define DIRECTION_PINx								GPIO_PIN_9 // use pin 8 PB8
#define STEP_PINx											GPIO_PIN_14

#define MS1_PIN_BASEx 								GPIOB
#define MS2_PIN_BASEx							 	  GPIOB
#define MS3_PIN_BASEx 								GPIOB
#define ENABLE_PIN_BASEx 							GPIOB
#define DIRECTION_PIN_BASEx 					GPIOB
#define STEP_PIN_BASEx 								GPIOB

#define MS1_PINy											GPIO_PIN_3 // pin 0 to be used with PB0 for Green LED
#define MS2_PINy											GPIO_PIN_7 // pin 7 for PB7 for Blue LED
#define MS3_PINy											GPIO_PIN_14 // pin 14 for pb14 connected to Red LED
#define ENABLE_PINy 									GPIO_PIN_1 // use pin 3 for enable
#define DIRECTION_PINy								GPIO_PIN_2 // use pin 2 PB2
#define STEP_PINy											GPIO_PIN_7

#define MS1_PIN_BASEy 								GPIOC
#define MS2_PIN_BASEy							 	  GPIOC
#define MS3_PIN_BASEy 								GPIOC
#define ENABLE_PIN_BASEy 							GPIOC
#define DIRECTION_PIN_BASEy 					GPIOC
#define STEP_PIN_BASEy 								GPIOB



// x and y axis motors
//17HS16-2004S1 Bipolar Stepper Motor
// 64oz.in (45 Ncm)
// 2.6mH
// 1.1ohm/phase
// 2A max current
// 1.8 deg per step (200 steps/rev at full step)
// minimum step time running at 24V .5A 0.217ms max speed = 345RPM
Big_Easy_Driver bedX; 
Big_Easy_Driver bedY;

void vMotorXTask(void *pvParameters);
void vMotorYTask(void *pvParameters);

void TIMER2_IRQHANDLER(void);
void TIMER3_IRQHANDLER(void);


int main(void)
{
		HAL_Init();
	  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0U);
		//HAL_NVIC_EnableIRQ(TIM2_IRQn);
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	TIM2_init();
	TIM3_init();

	GPIO_InitTypeDef DriverPins;
	
	DriverPins.Pin = GPIO_PIN_13;
	DriverPins.Mode = GPIO_MODE_INPUT;
	DriverPins.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &DriverPins);
	
	bedPinOut(&bedX, 
						MS1_PINx, 
						MS2_PINx,
						MS3_PINx,
						ENABLE_PINx, 
						DIRECTION_PINx,
						STEP_PINx);
	
	bedPinBase(&bedX, 
						 MS1_PIN_BASEx,
						 MS2_PIN_BASEx, 
						 MS3_PIN_BASEx, 
						 ENABLE_PIN_BASEx, 
						 DIRECTION_PIN_BASEx, 
						 STEP_PIN_BASEx);
						 
	bedPinOut(&bedY, 
						MS1_PINy, 
						MS2_PINy,
						MS3_PINy,
						ENABLE_PINy, 
						DIRECTION_PINy,
						STEP_PINy);
	
	bedPinBase(&bedY, 
						 MS1_PIN_BASEy,
						 MS2_PIN_BASEy, 
						 MS3_PIN_BASEy, 
						 ENABLE_PIN_BASEy, 
						 DIRECTION_PIN_BASEy, 
						 STEP_PIN_BASEy);
	
	  bedInit(&bedX, bedX.MsPinBase1, MS1_PINx|MS2_PINx|MS3_PINx|ENABLE_PINx|DIRECTION_PINx|STEP_PINx);
	  bedInit(&bedY, bedY.MsPinBase1, MS1_PINy|MS2_PINy|MS3_PINy|ENABLE_PINy|DIRECTION_PINy|STEP_PINy);
		bedInit(&bedY, bedY.StepPinBase, STEP_PINy);

		HAL_GPIO_WritePin(bedX.EnablePinBase, bedX.EnablePin, GPIO_PIN_RESET);
		setStepResolution(&bedX, FULL, MS1_PIN_BASEx);
		setDirection(&bedX, GPIO_PIN_SET, bedX.DirectionPinBase);
		
	while(1)
	{
		
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
		{
			//HAL_GPIO_WritePin(bedX.EnablePinBase, bedX.EnablePin, GPIO_PIN_RESET);
		 // bedMoveXinc(&bedX, 400, 1);
			
			
			//linearMoveXY(&bedX, &bedY, bedX.motor.position+2000, bedY.motor.position+2000, 10);
			linearMoveXY(&bedX, &bedY, bedX.motor.position-2, bedY.motor.position-20, 100);

			//HAL_GPIO_WritePin(bedX.EnablePinBase, bedX.EnablePin, GPIO_PIN_SET);
		}
		HAL_Delay(100);
	}
/*  bedInit(&bedX, bedX.MsPinBase1, MS1_PINx);
		bedInit(&bedX, bedX.MsPinBase2, MS2_PINx);
	 	bedInit(&bedX, bedX.MsPinBase3, MS3_PINx); 
*/



//	xTaskCreate(vMotorXTask,
//							"X-Axis Motor Controller",
//							100,
//							NULL,
//							1,
//							NULL
//						 );

//	xTaskCreate(vMotorYTask,
//							"Y-Axis Motor Controller",
//							100,
//							NULL,
//							1,
//							NULL
//						 );				

	
	
}
// Timer interrupt handlers to synchronize the X and Y motors (may move to RTOS tasks for better predictability)
void TIM2_IRQHandler(void)
{
	__disable_irq();
	if(TIM2->SR & TIM_SR_UIF) // if update event ocurred (though we know it was due to overflow in this case)
	{
		if(bedX.motor.position < bedX.motor.target_position)
		{
			HAL_GPIO_TogglePin(bedX.StepPinBase, bedX.StepPin);
			bedX.motor.position++;
		}
		
		else if(bedX.motor.position > bedX.motor.target_position)
		{
			HAL_GPIO_TogglePin(bedX.StepPinBase, bedX.StepPin);
			bedX.motor.position--;
		}
		else if(bedX.motor.position == bedX.motor.target_position)
		{
			TIM2->CR1 &= ~TIM_CR1_CEN;
		}
		
	}
		
	

	TIM2->SR &= ~TIM_SR_UIF;
	__enable_irq();
}

void TIM3_IRQHandler(void)
{
	__disable_irq();
	if(TIM3->SR & TIM_SR_UIF) // if update event ocurred (though we know it was due to overflow in this case)
	{
		if(bedY.motor.position < bedY.motor.target_position)
		{
			HAL_GPIO_TogglePin(bedY.StepPinBase, bedY.StepPin);
			bedY.motor.position++;
		}
		
		else if(bedY.motor.position > bedY.motor.target_position)
		{
			HAL_GPIO_TogglePin(bedY.StepPinBase, bedY.StepPin);
			bedY.motor.position--;
		}
		else if(bedY.motor.position == bedY.motor.target_position)
		{
			TIM3->CR1 &= ~TIM_CR1_CEN;
		}
		
	}
	
	TIM3->SR &= ~TIM_SR_UIF;
	__enable_irq();
}

void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}
 
