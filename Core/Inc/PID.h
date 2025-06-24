/*
 * PID.h
 *
 *  Created on: Oct 13, 2024
 *      Author: GIGABYTE
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>

#define USING_CLAMPING_SECTION 0
//let's begin

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float to;
	float T;//sampling time


	float et;// sai so tong
	float ek1;// sai so e(k-1)

	float eKit;
	float eKik1;

	float propotion;
	float integrator;
	float integratork1;
	float deviator;
	float deviatork1;

	float measurement;// input
	float expected;

	float upper_saturation;
	float lower_saturation;

	float v;//output before saturation
	float u;// output after saturation

	uint8_t uv;// using for integrator clamping (compare output before saturation and after saturation)
	uint8_t ev;//using for integrator clamping(compare error with output before saturation)

}PIDControllers_Typedef;


void pidControllersInit(PIDControllers_Typedef* pid,float Kp,float Ki,float Kd,float to,float T,float upper_satuaration,float lower_satuaration);
float pidUpdate(PIDControllers_Typedef* pid,float measurement,float input);
void AdjustPIDParams(PIDControllers_Typedef* pid,float Kp,float Ki,float Kd);
#ifdef __cplusplus
}
#endif
#endif /* INC_PID_H_ */
