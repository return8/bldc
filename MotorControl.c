/*
 * MotorControl.c
 *
 *  Created on: Jul 19, 2021
 *      Author: A
 */

#include "device.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>

extern uint8_t DEBUG_Hall;
extern uint8_t MOTOR_POLE_NUM;
extern uint8_t ADC_RESOLUTION;
extern uint8_t BREAK;
extern float SENSE_RESISTOR;

int PWM=0;
int ctr=0;

int Tstart;
int Tstop;
int Tdelta;


static void StopPWM(void);
static void PhaseAB(void);
static void PhaseAC(void);
static void PhaseBC(void);
static void PhaseBA(void);
static void PhaseCA(void);
static void PhaseCB(void);


void Hall_InterruptInit(){
	StopPWM();
	Tstart = HAL_GetTick();
}



/* Called by Hall effect sensor interrupt handler
 * Stop all PWM ,then start new
 * IntrptByHall = 1 -> hall interrupt
 * IntrptByHall = 0 -> speed control */
uint8_t Hall_ABC(uint8_t IntrptByHall) {

	StopPWM();

	uint8_t HC = HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_7);
	uint8_t HB = HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_8);
	uint8_t HA = HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_9);


	if (HA==0 && HB==0 && HC==1)           PhaseCB();
	else if (HA==0 && HB==1 && HC==0)      PhaseBA();
	else if (HA==0 && HB==1 && HC==1)      PhaseCA();
	else if (HA==1 && HB==0 && HC==0)      PhaseAC();
	else if (HA==1 && HB==0 && HC==1)      PhaseAB();
	else if (HA==1 && HB==1 && HC==0)      PhaseBC();
	else {
		if (DEBUG_Hall) printf("H > Hall sensor Error !!! \n");
		return 0;
	}

	if (IntrptByHall == 1){
		//interrupted by hall sensor
		//keep count of the current pole at ctr
		ctr++;
		//reset pole numbering at every full motor rotation
		if (ctr>=MOTOR_POLE_NUM){
			ctr=0;
			//calculate one full rotation time
			Tstop = HAL_GetTick();
			Tdelta = Tstop-Tstart;
			Tstart = HAL_GetTick();
		}
		return ctr-1;
	}

	/*interrupted by speed control*/
	return 0;
}

/*return motor RPM*/
int getRPM(){
	int rpm = 60000 /Tdelta;
	if ((HAL_GetTick()-Tstart) > 1000) rpm=0;
	return rpm;
}

/*return throttle input 0-255*/
uint8_t getThrottle(){
	//printf("adc %d \n",readADC1());
	PWM = (readADC1()*100)/ADC_RESOLUTION;
	return PWM;
}

/*return motor current */
float getCurrent(){
	int adcVal = readADC2();
	//printf("adc %d \n",adcVal);
	float volt=0.0;
	volt = ((float)adcVal*3.6)/4096;
	return (volt/SENSE_RESISTOR)*10;
}

uint8_t SpeedControl(){
	/* adjust PWM ,need implement by interrupt
	 * danger ISR when in Hall_ABC
	 */
	Hall_ABC(0);
}

static void PhaseAB(){
	//Phase1
	if(!BREAK){
		if (DEBUG_Hall) printf("H > PHASE AB 1\n");
		StartPWM_PB0(PWM);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);  //A1_Enable
		StartPWM_PB1N(PWM);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);  //A1_Enable
	}
	else{

	}
}

static void PhaseAC(){
	//Phase2
	if (DEBUG_Hall) printf("H > PHASE AC 2\n");
	StartPWM_PB0(PWM);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);  //A1_Enable
	StartPWM_PB11N(PWM);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  //A1_Enable

}

static void PhaseBC(){
	//Phase3
	if (DEBUG_Hall) printf("H > PHASE BC 3\n");
	StartPWM_PB1(PWM);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	StartPWM_PB11N(PWM);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
}

static void PhaseBA(){
	//Phase4
	if (DEBUG_Hall) printf("H > PHASE BA 4\n");
	StartPWM_PB1(PWM);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	StartPWM_PB0N(PWM);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
}

static void PhaseCA(){
	//Phase5
	if (DEBUG_Hall) printf("H > PHASE CA 5\n");
	StartPWM_PB11(PWM);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	StartPWM_PB0N(PWM);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
}

static void PhaseCB(){

	if (DEBUG_Hall) printf("H > PHASE CB 6\n");
	StartPWM_PB11(PWM);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	StartPWM_PB1N(PWM);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}


static void StopPWM(){
	StopPWM_PA1();
	StopPWM_PB1();
	StopPWM_PA7();
	StopPWM_PB0();
	StopPWM_PA6();
	StopPWM_PB11();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_10, GPIO_PIN_SET);

}
