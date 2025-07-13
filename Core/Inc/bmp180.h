/**
  ******************************************************************************
  * @file           : bmp180.h
  * @brief          : BMP180 Library for STM32 using I2C - Header File
  * @author         : Modified by Duong Quynh
  * @based_on       : ControllersTech
  * @date           : 13/07/2025
  ******************************************************************************
  * @license        : GNU GPL v3
  ******************************************************************************
*/

#ifndef __BMP180_H__
#define __BMP180_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define BMP180_I2C_ADDR (0x77 << 1)  // BMP180 8-bit address for HAL
#define BMP180_STD_ATM_PRESS 101325  // Standard atmospheric pressure in Pa

typedef struct {
    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t B1;
    int16_t B2;
    int16_t MB;
    int16_t MC;
    int16_t MD;
} BMP180_CalibData;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    BMP180_CalibData calib;
    uint8_t oss; // Oversampling setting: 0-3
    int32_t B5;  // Shared computation value
} BMP180_Handle_t;

// ===== Public API ===== //
HAL_StatusTypeDef BMP180_Init(BMP180_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t oss);
float BMP180_ReadTemperature(BMP180_Handle_t *dev);
float BMP180_ReadPressure(BMP180_Handle_t *dev);
float BMP180_ReadAltitude(BMP180_Handle_t *dev, float seaLevelPressure);

#endif // __BMP180_H__
