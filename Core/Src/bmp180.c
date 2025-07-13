#include "bmp180.h"
#include <math.h>
#include <string.h>  // For memset

static HAL_StatusTypeDef BMP180_ReadCalibrationData(BMP180_Handle_t *dev);
static int32_t BMP180_ReadUncompTemp(BMP180_Handle_t *dev);
static int32_t BMP180_ReadUncompPress(BMP180_Handle_t *dev);

HAL_StatusTypeDef BMP180_Init(BMP180_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t oss)
{
    if (!dev || !hi2c || oss > 3) return HAL_ERROR;

    dev->hi2c = hi2c;
    dev->oss = oss;
    dev->B5 = 0;

    return BMP180_ReadCalibrationData(dev);
}

static HAL_StatusTypeDef BMP180_ReadCalibrationData(BMP180_Handle_t *dev)
{
    uint8_t calib_data[22];
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Read(dev->hi2c, BMP180_I2C_ADDR, 0xAA, 1, calib_data, 22, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    dev->calib.AC1 = (int16_t)((calib_data[0] << 8) | calib_data[1]);
    dev->calib.AC2 = (int16_t)((calib_data[2] << 8) | calib_data[3]);
    dev->calib.AC3 = (int16_t)((calib_data[4] << 8) | calib_data[5]);
    dev->calib.AC4 = (uint16_t)((calib_data[6] << 8) | calib_data[7]);
    dev->calib.AC5 = (uint16_t)((calib_data[8] << 8) | calib_data[9]);
    dev->calib.AC6 = (uint16_t)((calib_data[10] << 8) | calib_data[11]);
    dev->calib.B1  = (int16_t)((calib_data[12] << 8) | calib_data[13]);
    dev->calib.B2  = (int16_t)((calib_data[14] << 8) | calib_data[15]);
    dev->calib.MB  = (int16_t)((calib_data[16] << 8) | calib_data[17]);
    dev->calib.MC  = (int16_t)((calib_data[18] << 8) | calib_data[19]);
    dev->calib.MD  = (int16_t)((calib_data[20] << 8) | calib_data[21]);

    return HAL_OK;
}

static int32_t BMP180_ReadUncompTemp(BMP180_Handle_t *dev)
{
    uint8_t cmd = 0x2E;
    uint8_t raw[2];

    HAL_I2C_Mem_Write(dev->hi2c, BMP180_I2C_ADDR, 0xF4, 1, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(5); // Datasheet: wait at least 4.5 ms

    HAL_I2C_Mem_Read(dev->hi2c, BMP180_I2C_ADDR, 0xF6, 1, raw, 2, HAL_MAX_DELAY);
    return ((raw[0] << 8) | raw[1]);
}

float BMP180_ReadTemperature(BMP180_Handle_t *dev)
{
    int32_t UT = BMP180_ReadUncompTemp(dev);
    int32_t X1 = ((UT - dev->calib.AC6) * dev->calib.AC5) >> 15;
    int32_t X2 = (dev->calib.MC << 11) / (X1 + dev->calib.MD);
    dev->B5 = X1 + X2;
    int32_t T = (dev->B5 + 8) >> 4;

    return T / 10.0f;
}

static int32_t BMP180_ReadUncompPress(BMP180_Handle_t *dev)
{
    uint8_t cmd = 0x34 + (dev->oss << 6);
    uint8_t raw[3];
    HAL_I2C_Mem_Write(dev->hi2c, BMP180_I2C_ADDR, 0xF4, 1, &cmd, 1, HAL_MAX_DELAY);

    switch (dev->oss)
    {
        case 0: HAL_Delay(5); break;
        case 1: HAL_Delay(8); break;
        case 2: HAL_Delay(14); break;
        case 3: HAL_Delay(26); break;
    }

    HAL_I2C_Mem_Read(dev->hi2c, BMP180_I2C_ADDR, 0xF6, 1, raw, 3, HAL_MAX_DELAY);

    return (((int32_t)raw[0] << 16) | ((int32_t)raw[1] << 8) | raw[2]) >> (8 - dev->oss);
}

float BMP180_ReadPressure(BMP180_Handle_t *dev)
{
    int32_t UP = BMP180_ReadUncompPress(dev);
    int32_t B6 = dev->B5 - 4000;

    int32_t X1 = (dev->calib.B2 * ((B6 * B6) >> 12)) >> 11;
    int32_t X2 = (dev->calib.AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = ((((int32_t)dev->calib.AC1 * 4 + X3) << dev->oss) + 2) >> 2;

    X1 = (dev->calib.AC3 * B6) >> 13;
    X2 = (dev->calib.B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    uint32_t B4 = (dev->calib.AC4 * (uint32_t)(X3 + 32768)) >> 15;
    uint32_t B7 = ((uint32_t)UP - B3) * (50000 >> dev->oss);

    int32_t P;
    if (B7 < 0x80000000)
        P = (B7 << 1) / B4;
    else
        P = (B7 / B4) << 1;

    X1 = (P >> 8) * (P >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * P) >> 16;
    P = P + ((X1 + X2 + 3791) >> 4);

    return (float)P; // in Pa
}

float BMP180_ReadAltitude(BMP180_Handle_t *dev, float seaLevelPressure)
{
    float pressure = BMP180_ReadPressure(dev);  // in Pa
    return 44330.0f * (1.0f - powf(pressure / seaLevelPressure, 0.1903f));
}
