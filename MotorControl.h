/*
 * MotorControl.h
 *
 *  Created on: Jul 19, 2021
 *      Author: A
 */

#ifndef INC_MOTORCONTROL_H_
#define INC_MOTORCONTROL_H_

void Hall_InterruptInit();
int getRPM();
uint8_t  Hall_ABC(uint8_t IntrptByHall);
uint8_t getThrottle();
uint8_t SpeedControl();
float getCurrent();


#endif /* INC_MOTORCONTROL_H_ */
