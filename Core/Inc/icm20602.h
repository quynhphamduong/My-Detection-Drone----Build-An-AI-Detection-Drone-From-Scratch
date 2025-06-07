/*
 * icm20602.h
 *
 *  Created on: Feb 6, 2025
 *  Author: ADMIN
 */

#ifndef INC_ICM20602_H_
#define INC_ICM20602_H_
#ifdef __cplusplus
 extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include "main.h"
 /**
 * @brief ICM20602 SPI macro
 */
#define CS_GPIO_Port          GPIOB
#define CS_Pin                GPIO_PIN_0

#define RAD_TO_DEG            57.2958   // Macro to convert radian to degree
#define ICM20602_READ_BIT     0x80
#define ICM20602_WRITE_BIT    0x7F


 typedef struct ICM20602{
 	short acc_x_raw;
 	short acc_y_raw;
 	short acc_z_raw;

 	short gyro_x_raw;
 	short gyro_y_raw;
 	short gyro_z_raw;

 	float acc_x;
 	float acc_y;
 	float acc_z;

 	float gyro_x;
 	float gyro_y;
 	float gyro_z;

	float CF_roll;
	float CF_pitch;

	float CF_roll_degree;
	float CF_pitch_degree;
 }Struct_ICM20602;



 /**
  * @brief ICM20602 Register Map
  */
 #define	XG_OFFS_TC_H	0x04
 #define	XG_OFFS_TC_L	0x05
 #define	YG_OFFS_TC_H	0x07
 #define	YG_OFFS_TC_L	0x08
 #define	ZG_OFFS_TC_H	0x0A
 #define	ZG_OFFS_TC_L	0x0B
 #define	SELF_TEST_X_ACCEL	0x0D
 #define	SELF_TEST_Y_ACCEL	0x0E
 #define	SELF_TEST_Z_ACCEL	0x0F
 #define	XG_OFFS_USRH	0x13
 #define	XG_OFFS_USRL	0x14
 #define	YG_OFFS_USRH	0x15
 #define	YG_OFFS_USRL	0x16
 #define	ZG_OFFS_USRH	0x17
 #define	ZG_OFFS_USRL	0x18
 #define	SMPLRT_DIV	0x19
 #define	CONFIG	0x1A    //The default value of the register is 0x80.
 #define	GYRO_CONFIG	0x1B
 #define	ACCEL_CONFIG	0x1C
 #define	ACCEL_CONFIG2	0x1D
 #define	LP_MODE_CFG	0x1E
 #define	ACCEL_WOM_X_THR	0x20
 #define	ACCEL_WOM_Y_THR	0x21
 #define	ACCEL_WOM_Z_THR	0x22
 #define	FIFO_EN	0x23
 #define	FSYNC_INT	0x36
 #define	INT_PIN_CFG	0x37
 #define	INT_ENABLE	0x38
 #define	FIFO_WM_INT_STATUS	0x39

 #define	INT_STATUS	0x3A
 #define	ACCEL_XOUT_H	0x3B
 #define	ACCEL_XOUT_L	0x3C
 #define	ACCEL_YOUT_H	0x3D
 #define	ACCEL_YOUT_L	0x3E
 #define	ACCEL_ZOUT_H	0x3F
 #define	ACCEL_ZOUT_L	0x40
 #define	TEMP_OUT_H	0x41
 #define	TEMP_OUT_L	0x42
 #define	GYRO_XOUT_H	0x43
 #define	GYRO_XOUT_L	0x44
 #define	GYRO_YOUT_H	0x45
 #define	GYRO_YOUT_L	0x46
 #define	GYRO_ZOUT_H	0x47
 #define	GYRO_ZOUT_L	0x48
 #define	SELF_TEST_X_GYRO	0x50
 #define	SELF_TEST_Y_GYRO	0x51
 #define	SELF_TEST_Z_GYRO	0x52
 #define	FIFO_WM_TH1	0x60
 #define	FIFO_WM_TH2	0x61
 #define	SIGNAL_PATH_RESET	0x68
 #define	ACCEL_INTEL_CTRL	0x69
 #define	USER_CTRL	0x6A
 #define	PWR_MGMT_1	0x6B //The default value of the register is 0x41.
 #define	PWR_MGMT_2	0x6C
 #define	I2C_IF	0x70
 #define	FIFO_COUNTH	0x72
 #define	FIFO_COUNTL	0x73
 #define	FIFO_R_W	0x74
 #define	WHO_AM_I	0x75 //The default value of the register is 0x12. but actually i receive 0x09
 #define	XA_OFFSET_H	0x77
 #define	XA_OFFSET_L	0x78
 #define	YA_OFFSET_H	0x7A
 #define	YA_OFFSET_L	0x7B
 #define	ZA_OFFSET_H	0x7D
 #define	ZA_OFFSET_L	0x7E

 /**
  * @brief ICM20602 SPI Function definition.
  */
   int  icm20602_init(void);
   void icm20602_readReg(uint8_t addr_reg, uint8_t *data, uint8_t len);
   void icm20602_writeReg(uint8_t addr_reg, uint8_t data);
	 void get3AxisAccelData(Struct_ICM20602 *imu);
	 void get3AxisGyroData(Struct_ICM20602 *imu);
	 void icm20602CalculateAngle(Struct_ICM20602 *imu, float delta_time);

#ifdef __cplusplus
}
#endif


#endif /* INC_ICM20602_H_ */
