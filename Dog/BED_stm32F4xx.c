#include "BED_stm32F4xx.h"




void bedPinOut(Big_Easy_Driver* bed, 
							 uint32_t Ms1, 
							 uint32_t Ms2,
							 uint32_t Ms3, 
							 uint32_t En,
							 uint32_t dir,
							 uint32_t step){
																bed->MsPin1 = Ms1;
																bed->MsPin2 = Ms2;
																bed->MsPin3 = Ms3;
																bed->EnablePin = En;
																bed->DirectionPin = dir;
																bed->StepPin = step;
														 }
							 
														 
														 
														 
void bedPinBase(Big_Easy_Driver* bed, 
								GPIO_TypeDef* Ms1Base,
								GPIO_TypeDef* Ms2Base,
								GPIO_TypeDef* Ms3Base,
								GPIO_TypeDef* EnableBase,
								GPIO_TypeDef* DirectionBase,
								GPIO_TypeDef* StepBase){
																				bed->MsPinBase1 = Ms1Base;
																				bed->MsPinBase2 = Ms2Base;
																				bed->MsPinBase3 = Ms3Base;
																				bed->EnablePinBase = EnableBase;
																				bed->DirectionPinBase = DirectionBase;
																				bed->StepPinBase = StepBase;
																			}

																																				

void bedOptionalPinOut(Big_Easy_Driver* bed, uint32_t rst, uint32_t slp)
{
	bed->ResetPin = rst;
	bed->SleepPin = slp;
}




void bedInit(Big_Easy_Driver* bed, GPIO_TypeDef* GPIOx, uint32_t pins)
{
	GPIO_InitTypeDef DriverPins;
	
	DriverPins.Pin = pins;
	DriverPins.Mode = GPIO_MODE_OUTPUT_PP;
	DriverPins.Pull = GPIO_NOPULL;
	DriverPins.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &DriverPins);
}


void setDirection(Big_Easy_Driver* bed, GPIO_PinState dir, GPIO_TypeDef* GPIOx)
{
	HAL_GPIO_WritePin(GPIOx, bed->DirectionPin, dir);
}





void setStepResolution(Big_Easy_Driver* bed, int resolution, GPIO_TypeDef* GPIOx)
{
	switch(resolution)
	{
		case FULL: // L L L
			HAL_GPIO_WritePin(GPIOx,bed->MsPin1|bed->MsPin2|bed->MsPin3,GPIO_PIN_RESET);
			break;
		
		case HALF: // H L L
			HAL_GPIO_WritePin(GPIOx,bed->MsPin2|bed->MsPin3,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOx,bed->MsPin1,GPIO_PIN_SET);
			break;
		
		case QUARTER: // L H L
			HAL_GPIO_WritePin(GPIOx,bed->MsPin1|bed->MsPin3,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOx,bed->MsPin2,GPIO_PIN_SET);
			break;
		
		case EIGHTH: // H H L
			HAL_GPIO_WritePin(GPIOx,bed->MsPin3,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOx,bed->MsPin1|bed->MsPin2,GPIO_PIN_SET);
			break;
		
		case SIXTEENTH: // H H H
			HAL_GPIO_WritePin(GPIOx,bed->MsPin1|bed->MsPin2|bed->MsPin3,GPIO_PIN_SET);
			break;
	}
}


void bedMoveXinc(Big_Easy_Driver* bed, double x_inc, double speed)
{
	if(x_inc >= 0)
		HAL_GPIO_WritePin(bed->DirectionPinBase, bed->DirectionPin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(bed->DirectionPinBase, bed->DirectionPin, GPIO_PIN_RESET);

	// possible to queue steps into DMA for quick access and possible lookahead features?
	double finalPosition = bed->motor.position+x_inc;
	
	
	TIM2->CR1 |= TIM_CR1_CEN;
	while(bed->motor.position != finalPosition)
	{
	 // motor turns until it reaches final position
	}
	TIM2->CR1 &= ~TIM_CR1_CEN;
}

void bedMoveYinc(Big_Easy_Driver* bed, double y_inc, double speed)
{
	// not needed 
}

void bedMoveXabs(Big_Easy_Driver* bed, double x_pos, double speed)
{
}

void bedMoveYabs(Big_Easy_Driver* bed, double y_pos, double speed)
{
		// not needed only one function needed to handle single motor operations

}
void linearMoveXY(Big_Easy_Driver* bedX, Big_Easy_Driver* bedY, double x_pos, double y_pos, double speed)
{
	double dX = x_pos - bedX->motor.position;
	double dY = y_pos - bedY->motor.position;
	double dS = sqrt(pow(dX,2.0)+pow(dY, 2.0));
	
	double ratioX = fabs(dX/dS);
	double ratioY = fabs(dY/dS);
	
	
	//(3/5*10+4/5*10)
	//speed = pulse/sec * pitch/pulse
	// moving X and Y with respect to their ratios related to change in distance
	// will translate to speed of X and speed of Y
	double speedY = ratioX*speed;
	double speedX = ratioY*speed;
	
	bedX->motor.target_position = x_pos;
	bedY->motor.target_position = y_pos;
	
	if (dX>0)
	{
		setDirection(bedX, GPIO_PIN_SET, bedX->DirectionPinBase);
	}
	else
	{
		setDirection(bedX, GPIO_PIN_RESET, bedX->DirectionPinBase);
	}
	
	if (dY>0)
	{
		setDirection(bedY, GPIO_PIN_SET, bedY->DirectionPinBase);
	}
	else
	{
		setDirection(bedY, GPIO_PIN_RESET, bedY->DirectionPinBase);
	}
	
	TIM2->ARR = speedX-1;
	TIM3->ARR = speedY-1;
	
	TIM2->EGR |= TIM_EGR_UG; 
	TIM3->EGR |= TIM_EGR_UG; 
	
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM3->CR1 |= TIM_CR1_CEN;

}

void circularInterp(Big_Easy_Driver* bedX, Big_Easy_Driver* bedY, double x_pos, double y_pos, double rad, double speed)
{
	
}


void TIM2_init()
{
		// enable TIM2 clock gate
		__HAL_RCC_TIM2_CLK_ENABLE();
		//SystemCoreClock = 180MHz
		//PSC = 180/2/10000-1 = 9000-1
		TIM2->PSC = (uint32_t)((SystemCoreClock *.00005)) - 1;// prescaler (note that 0 is /1) (divided by 2 because we need an on and off pulse for stepper actuation)
		//TIM2->ARR = 100-1;                    // reload on overflow (pulses/s = 10000/TIM2->ARR)
		TIM2->CR1 |= TIM_CR1_URS; 						// generate update on overflow/underflow 
		TIM2->DIER = TIM_DIER_UIE;            // overflow isr
		TIM2->EGR |= TIM_EGR_UG;              //update generation 
		//TIM2->CR1 |= TIM_CR1_CEN;						// enable timer do this when function called
		
		HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM3_init()
{
		// enable TIM2 clock gate
		__HAL_RCC_TIM3_CLK_ENABLE();
		//SystemCoreClock = 180MHz
		//PSC = 180/2/10000-1 = 9000-1
		TIM3->PSC = (uint32_t)((SystemCoreClock *.00005)) - 1;// prescaler (note that 0 is /1) (divided by 2 because we need an on and off pulse for stepper actuation)
		//TIM3->ARR = 100-1;                    // reload on overflow (pulses/s = 10000/TIM2->ARR)
		TIM3->CR1 |= TIM_CR1_URS; 						// generate update on overflow/underflow 
		TIM3->DIER = TIM_DIER_UIE;            // overflow isr
		TIM3->EGR |= TIM_EGR_UG;              //update generation 
		//TIM3->CR1 |= TIM_CR1_CEN;						// enable timer do this when function called
		
		HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM3_IRQn);
}


 
