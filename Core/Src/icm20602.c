/**
 * File: ICM20602.c
 * @author Pham Duong Quynh
 * This library is used for STM32, only SPI protocol, using HAL driver.
 *
 * version : 1.0
 * January, 2025
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "main.h"
#include "ICM20602.h"
#include "stm32f4xx_hal.h"


extern SPI_HandleTypeDef hspi1;


int  icm20602_init(void)
{
	//Checking device
	//uint8_t who_am_i = 0 ;
	//icm20602_readReg(WHO_AM_I, &who_am_i, 1);

	//Configuration device
	  // Reset sensor
    icm20602_writeReg(PWR_MGMT_1, 0x80);
    HAL_Delay(50);

    //  PLL voi xung nhip X
    icm20602_writeReg(PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    // Configure gyro (2000dps)
	  icm20602_writeReg(CONFIG, 0x05); // Gyro LPF fc 20Hz(bit2:0-100) at 1kHz sample rate
	  HAL_Delay(50);
    icm20602_writeReg(GYRO_CONFIG, 0x18); //Gyro sensitivity 2000 dps, FCHOICE (bit1:0-00)
	  HAL_Delay(50);

    // Configure accel (16g)
    icm20602_writeReg(ACCEL_CONFIG, 0x18); // Acc sensitivity 16g
	  HAL_Delay(50);
	  icm20602_writeReg(ACCEL_CONFIG2, 0x03); // Acc FCHOICE 1kHz(bit3-0), DLPF fc 44.8Hz(bit2:0-011)
	  HAL_Delay(50);

		// Enable Acc(bit5:3-000), Enable Gyro(bit2:0-000)
		icm20602_writeReg( PWR_MGMT_2, 0x00 );
		HAL_Delay(50);

	  // Enable Interrupts when data is ready
	  icm20602_writeReg(INT_ENABLE, 0x01); // Enable DRDY Interrupt
	  HAL_Delay(50);

    // Sample rate 1 kHz
    icm20602_writeReg(SMPLRT_DIV, 0x07);
		HAL_Delay(50);


    return 0; // Success

}
void icm20602_readReg(uint8_t addr_reg, uint8_t *data, uint8_t len)
{
	  uint8_t address_reg = ICM20602_READ_BIT | addr_reg;// example 0x80|0x75 = 0xF5
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); //CS down
	  HAL_SPI_Transmit(&hspi1, &address_reg, 1, 100);
	  HAL_SPI_Receive(&hspi1, data, len, 100);
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);  //CS high back

}

void icm20602_writeReg(uint8_t addr_reg, uint8_t data)
{
	uint8_t address_reg = ICM20602_WRITE_BIT & addr_reg;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); //CS down
	HAL_SPI_Transmit(&hspi1, &address_reg, 1, 100);
	HAL_SPI_Transmit(&hspi1, &data, 1, 100);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); //CS high back
}

void get3AxisAccelData(Struct_ICM20602 *imu)
{
	uint8_t data[10];  //data receive from ICM20602
	uint8_t address_reg = ICM20602_READ_BIT | ACCEL_XOUT_H;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); //CS down
	HAL_SPI_Transmit(&hspi1, &address_reg, 1, 100);
	HAL_SPI_Receive(&hspi1, data, 6, 100);

	(imu->acc_x_raw)= (int16_t) data[0]<<8 | data[1];
	(imu->acc_y_raw)= (int16_t) data[2]<<8 | data[3];
	(imu->acc_z_raw)= (int16_t) data[4]<<8 | data[5];

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); //CS high back

	/*Convert raw*/
  imu->acc_x = (float)imu->acc_x_raw * 16.0 / 32768.0; //  g
  imu->acc_y = (float)imu->acc_y_raw * 16.0 / 32768.0; //  g
  imu->acc_z = (float)imu->acc_z_raw * 16.0 / 32768.0; //  g
}


void get3AxisGyroData(Struct_ICM20602 *imu)
{
	uint8_t data[6];  //data receive from ICM20602
	uint8_t address_reg = ICM20602_READ_BIT |GYRO_XOUT_H;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); //CS down
	HAL_SPI_Transmit(&hspi1, &address_reg, 1, 100);
	HAL_SPI_Receive(&hspi1, data, 6, 100);

	(imu->gyro_x_raw)= (int16_t) data[0]<<8 | data[1];
	(imu->gyro_y_raw)= (int16_t) data[2]<<8 | data[3];
	(imu->gyro_z_raw)= (int16_t) data[4]<<8 | data[5];

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); //CS high back

	/*Convert raw */
  imu->gyro_x = (float)imu->gyro_x_raw * 2000.0 / 32768.0; //  dps
  imu->gyro_y = (float)imu->gyro_y_raw * 2000.0 / 32768.0; //  dps
  imu->gyro_z = (float)imu->gyro_z_raw * 2000.0 / 32768.0; //  dps

}

void icm20602CalculateAngle(Struct_ICM20602 *imu, float delta_time)
{
    float alpha = 0.98; // He so trong so cho gyro (98% gyro, 2% accel)

    float accel_roll = atan2(imu->acc_y, sqrt(pow(imu->acc_x,2)+ pow(imu->acc_z,2))) ;
    float accel_pitch = atan2(-imu->acc_x, sqrt(pow(imu->acc_y,2) + pow(imu->acc_z,2))) ;

    // Bo loc Complementary de tính góc Roll và Pitch
    imu->CF_roll = alpha * (imu->CF_roll + imu->gyro_x * delta_time) + (1 - alpha) * accel_roll;
    imu->CF_pitch = alpha * (imu->CF_pitch + imu->gyro_y * delta_time) + (1 - alpha) * accel_pitch;

	  //Convert to degree
	  imu->CF_roll_degree =  (imu->CF_roll)*RAD_TO_DEG;
	  imu->CF_pitch_degree = (imu->CF_pitch)*RAD_TO_DEG;

   // Thieu goc yaw !!
}

