/*
 * Drone_control.h
 *
 *  Created on: Jun 7, 2025
 *      Author: GIGABYTE
 */

#ifndef INC_DRONE_CONTROL_H_
#define INC_DRONE_CONTROL_H_
#include <stdint.h>
#include "PID.h"


typedef struct
{
	uint16_t RH; //Right Horizontal
	uint16_t RV; //Right Vertical
	uint16_t LV; //Left Vertical
	uint16_t LH; //Left Horizontal
/*
 * These values are set by controller and the controller values are
 * transmit through SPI
 */

	uint16_t SwA;
	uint16_t SwB;
	uint16_t SwC;
	uint16_t SwD;
	uint16_t VrA;
	uint16_t VrB;

	uint8_t FailSafe;
}Drone_Control_Typedef;

typedef struct
{
	float picth_reference;
	float roll_reference;
	float yaw_reference;

	float picth_measurement;
	float roll_measurement;
	float yaw_measurement;

	float picth_rate_reference;
	float roll_rate_reference;
	float yaw_rate_reference;

	float picth_rate_measurement;
	float roll_rate_measurement;
	float yaw_rate_measurement;

	float high_reference;
	float high_measurement;

}Drone_Calculation_Typedef;

void Calculate_Reference(Drone_Calculation_Typedef *dr,Drone_Control_Typedef *dc);
#endif /* INC_DRONE_CONTROL_H_ */
