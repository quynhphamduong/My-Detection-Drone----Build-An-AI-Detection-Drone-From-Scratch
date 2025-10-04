/*
 * Drone_control.c
 *
 *  Created on: Jun 7, 2025
 *      Author: GIGABYTE
 */

#include "Drone_control.h"

void Calculate_Reference(Drone_Calculation_Typedef *dr,Drone_Control_Typedef *dc)
{
	dr->picth_reference=(dc->RV-1500)*0.1f;//we can send reference direct from control
	dr->roll_reference=(dc->RH-1500)*0.1f;
	dr->yaw_reference=(dc->LH-1500)*0.1f;
	dr->high_reference=(dc->LV-1000)*10;
}


