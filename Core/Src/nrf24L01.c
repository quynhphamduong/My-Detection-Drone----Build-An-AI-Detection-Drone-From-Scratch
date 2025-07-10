/*
 * nRF24L01.c
 *
 *  Created on: Jan 24, 2025
 *      Author: GIGABYTE
 */

#include "nRF24L01.h"
#include "main.h"

void Chip_Select(NRF_HandleTypeDef *nrf)
{
	HAL_GPIO_WritePin(nrf->CS_GPIO, nrf->CS_PIN, 0);
}

void Chip_Deselect(NRF_HandleTypeDef *nrf)
{
	HAL_GPIO_WritePin(nrf->CS_GPIO, nrf->CS_PIN, 1);
}

void Set_CE_High(NRF_HandleTypeDef *nrf)
{
	HAL_GPIO_WritePin(nrf->CE_GPIO, nrf->CE_PIN, 1);
}

void Set_CE_Low(NRF_HandleTypeDef *nrf)
{
	HAL_GPIO_WritePin(nrf->CE_GPIO, nrf->CE_PIN, 0);
}

void WaitForIRQ(NRF_HandleTypeDef *nrf)
{
	while (HAL_GPIO_ReadPin(nrf->IRQ_GPIO, nrf->IRQ_PIN) == 1)
		;
}

void nRF_WriteRegister(NRF_HandleTypeDef *nrf, uint8_t reg, uint8_t *data, int size)
{
	uint8_t buff[size + 1];
	buff[0] = reg | (1 << 5);
	for (int i = 0; i < size; i++)
	{
		buff[i + 1] = data[i];
	}
	Chip_Select(nrf);
	HAL_SPI_Transmit(nrf->hspi, buff, (uint16_t)size + 1, NRF_SPI_TIMEOUT);
	Chip_Deselect(nrf);
}

void nRF_WriteOneRegister(NRF_HandleTypeDef *nrf, uint8_t reg, uint8_t data)
{
	uint8_t buff[2];
	buff[0] = reg | (1 << 5);
	buff[1] = data;
	Chip_Select(nrf);
	HAL_SPI_Transmit(nrf->hspi, buff, 2, NRF_SPI_TIMEOUT);
	Chip_Deselect(nrf);
}

void nRF_ReadRegister(NRF_HandleTypeDef *nrf, uint8_t reg, uint8_t *receive_data, uint16_t size)
{

	uint8_t buff = reg;
	Chip_Select(nrf);
	HAL_SPI_Transmit(nrf->hspi, &buff, 1, NRF_SPI_TIMEOUT);
	HAL_SPI_Receive(nrf->hspi, receive_data, size, NRF_SPI_TIMEOUT * 10);
	Chip_Deselect(nrf);
}

void nRF_ReadOneRegister(NRF_HandleTypeDef *nrf, uint8_t reg, uint8_t *receive_data)
{

	uint8_t buff = reg;
	Chip_Select(nrf);
	HAL_SPI_Transmit(nrf->hspi, &buff, 1, NRF_SPI_TIMEOUT);
	HAL_SPI_Receive(nrf->hspi, receive_data, 1, NRF_SPI_TIMEOUT);
	Chip_Deselect(nrf);
}
void nRF_SendCmd(NRF_HandleTypeDef *nrf, uint8_t cmd)
{

	uint8_t buff = cmd;
	Chip_Select(nrf);
	HAL_SPI_Transmit(nrf->hspi, &buff, 1, NRF_SPI_TIMEOUT);
	Chip_Deselect(nrf);
}

uint8_t nRF_GetStatus(NRF_HandleTypeDef *nrf)
{

	uint8_t buff = STATUS;
	uint8_t rx_data = 0;
	Chip_Select(nrf);
	HAL_SPI_Transmit(nrf->hspi, &buff, 1, NRF_SPI_TIMEOUT);
	HAL_SPI_Receive(nrf->hspi, &rx_data, 1, NRF_SPI_TIMEOUT * 10);
	Chip_Deselect(nrf);
	return rx_data;
}

void nRF_TX_Payload(NRF_HandleTypeDef *nrf, uint8_t *data, uint16_t size)
{
	Chip_Select(nrf);
	uint8_t buff = W_TX_PAYLOAD;
	HAL_SPI_Transmit(nrf->hspi, &buff, 1, NRF_SPI_TIMEOUT);
	HAL_SPI_Transmit(nrf->hspi, data, size, NRF_SPI_TIMEOUT);
	Chip_Deselect(nrf);
}

void nRF_RX_Payload(NRF_HandleTypeDef *nrf, uint8_t *rx_data, uint16_t size)
{
	Chip_Select(nrf);
	uint8_t buff = R_RX_PAYLOAD;
	HAL_SPI_Transmit(nrf->hspi, &buff, 1, NRF_SPI_TIMEOUT);
	HAL_SPI_Receive(nrf->hspi, rx_data, size, NRF_SPI_TIMEOUT * 10);
	Chip_Deselect(nrf);
}

void TX_Enhanced_ShockBurst_Config(NRF_HandleTypeDef *nrf,uint64_t tx_addr)
{
	uint8_t buff = 0x0a;
	Set_CE_Low(nrf);
	TX_ADDR_Write(nrf, tx_addr);
	nRF_WriteOneRegister(nrf, EN_AA, 0x00);
	nRF_WriteOneRegister(nrf, RX_PW_P0, 32);
	nRF_WriteOneRegister(nrf, RF_SETUP, 0x7);
	CONFIG_REG_Write(nrf, buff);
	Set_CE_High(nrf);
	HAL_Delay(2);
	nrf->nrfmode = MODE_TX;
}

void Select_Tx_Mode(NRF_HandleTypeDef *nrf)
{
	uint8_t buff = 0x0a;
	Set_CE_Low(nrf);
	CONFIG_REG_Write(nrf, buff);
	Set_CE_High(nrf);
	HAL_Delay(2);
	nrf->nrfmode = MODE_TX;
}

void RX_Enhanced_ShockBurst_Config(NRF_HandleTypeDef *nrf,uint64_t rx_addr)
{
	uint8_t buff = 0xb;
	Set_CE_Low(nrf);
	RX_ADDR_P0_Write(nrf,rx_addr);
	nRF_WriteOneRegister(nrf, EN_RXADDR, 0x01);
	nRF_WriteOneRegister(nrf, RX_PW_P0, 32);
	nRF_WriteOneRegister(nrf, EN_AA, 0x00);
	nRF_WriteOneRegister(nrf, RF_SETUP, 0x7);
	CONFIG_REG_Write(nrf, buff);
	Set_CE_High(nrf);
	HAL_Delay(2);
	nRF_SendCmd(nrf, FLUSH_RX);
	nrf->nrfmode = MODE_RX;
}

void Select_Rx_Mode(NRF_HandleTypeDef *nrf)
{
	uint8_t buff = 0x0b;
	Set_CE_Low(nrf);
	CONFIG_REG_Write(nrf, buff);
	Set_CE_High(nrf);
	HAL_Delay(2);
	nrf->nrfmode = MODE_RX;
}

#if USING_NRF_RTOS==1
void TX_Enhanced_ShockBurst_Config_RTOS(NRF_HandleTypeDef *nrf)
{
	uint8_t buff = 0x0a;
	Set_CE_Low(nrf);
	CONFIG_REG_Write(nrf, buff);
	nRF_WriteOneRegister(nrf, EN_AA, 0x00);
	nRF_WriteOneRegister(nrf, RF_SETUP, 0x7);
	Set_CE_High(nrf);
	vTaskDelay(pdMS_TO_TICKS(2));
	nrf->nrfmode = MODE_TX;
}

void Select_Tx_Mode_RTOS(NRF_HandleTypeDef *nrf)
{
	uint8_t buff = 0x0a;
	Set_CE_Low(nrf);
	CONFIG_REG_Write(nrf, buff);
	Set_CE_High(nrf);
	vTaskDelay(pdMS_TO_TICKS(2));
	nrf->nrfmode = MODE_TX;
}

void RX_Enhanced_ShockBurst_Config_RTOS(NRF_HandleTypeDef *nrf)
{
	uint8_t buff = 0xb;
	Set_CE_Low(nrf);
	CONFIG_REG_Write(nrf, buff);
	nRF_WriteOneRegister(nrf, EN_RXADDR, 0x01);
	nRF_WriteOneRegister(nrf, RX_PW_P0, 32);
	nRF_WriteOneRegister(nrf, EN_AA, 0x00);
	nRF_WriteOneRegister(nrf, RF_SETUP, 0x7);
	Set_CE_High(nrf);
	vTaskDelay(pdMS_TO_TICKS(2));
	nRF_SendCmd(nrf, FLUSH_RX);
	nrf->nrfmode = MODE_RX;
}

void Select_Rx_Mode_RTOS(NRF_HandleTypeDef *nrf)
{
	uint8_t buff = 0x0b;
	Set_CE_Low(nrf);
	CONFIG_REG_Write(nrf, buff);
	Set_CE_High(nrf);
	vTaskDelay(pdMS_TO_TICKS(2));
	nrf->nrfmode = MODE_RX;
}

void Two_Way_Commuination_RTOS(NRF_HandleTypeDef *nrf,uint8_t *tx_data,uint8_t *rx_data)
{
	TX_Communication(nrf,tx_data);
	Select_Rx_Mode_RTOS(nrf);
	vTaskDelay(pdMS_TO_TICKS(50));
	RX_Communication(nrf,rx_data);
	Select_Tx_Mode_RTOS(nrf);
	vTaskDelay(pdMS_TO_TICKS(10));
}

#endif

void Two_Way_Commuination_Pipe0_Config(NRF_HandleTypeDef *nrf, uint64_t tx_addr, uint64_t rx_addr)
{
	Set_CE_Low(nrf);
	RX_PW_P_NUM_Number_Of_Bytes(nrf, 0, 32);
	TX_ADDR_Write(nrf, tx_addr);
	RX_ADDR_P0_Write(nrf, rx_addr);
	nRF_WriteOneRegister(nrf, EN_RXADDR, 1);
	nRF_WriteOneRegister(nrf, EN_AA, 0x00);
	nRF_WriteOneRegister(nrf, RF_SETUP, 0x7);
	CONFIG_REG_Write(nrf, 0xa);
	Set_CE_High(nrf);
	HAL_Delay(2);
}

void Two_Way_Commuination_Pipe1_Config(NRF_HandleTypeDef *nrf, uint64_t tx_addr, uint64_t rx_addr)
{
	Set_CE_Low(nrf);
	RX_PW_P_NUM_Number_Of_Bytes(nrf, 1, 32);
	RX_ADDR_P1_Write(nrf, rx_addr);
	TX_ADDR_Write(nrf, tx_addr);
	nRF_WriteOneRegister(nrf, EN_RXADDR, 2);
	nRF_WriteOneRegister(nrf, EN_AA, 0x00);
	nRF_WriteOneRegister(nrf, RF_SETUP, 0x7);
	CONFIG_REG_Write(nrf, 0xa);
	Set_CE_High(nrf);
	HAL_Delay(2);
}

uint8_t TX_Communication(NRF_HandleTypeDef *nrf, uint8_t *data)
{
	if (nrf->nrfmode == MODE_TX)
	{
		nRF_TX_Payload(nrf, data, 32);
		Set_CE_High(nrf);
		WaitForIRQ(nrf);
		nRF_SendCmd(nrf, FLUSH_TX);
		uint8_t status = nRF_GetStatus(nrf);
		if ((status & (1 << MAX_RT)) != 0)
		{
			status |= ((1 << MAX_RT) | (1 << TX_FULL));
			nRF_WriteOneRegister(nrf, STATUS, status);
			return STATUS_TX_ERROR;
		}
		else if ((status & (1 << 5)) != 0)
		{
			status |= ((1 << TX_DS) | (1 << TX_FULL));
			nRF_WriteOneRegister(nrf, STATUS, status);
			return STATUS_TX_OK;
		}
	}
	return STATUS_TX_NONDEFINE;
}

uint8_t RX_Communication(NRF_HandleTypeDef *nrf, uint8_t *rx_data)
{
	if (nrf->nrfmode == MODE_RX)
	{
		Set_CE_High(nrf);
		uint8_t status = nRF_GetStatus(nrf);
		if ((status & (1 << RX_DR)) != 0)
		{
			nRF_WriteOneRegister(nrf, STATUS, (1 << 6));
			nRF_RX_Payload(nrf, rx_data, 32);
			return STATUS_RX_OK;
		}
		nRF_SendCmd(nrf, FLUSH_RX);
		return STATUS_RX_ERROR;
	}
	return STATUS_RX_NONEDEFINE;
}

void Two_Way_Commuination(NRF_HandleTypeDef *nrf,uint8_t *tx_data,uint8_t *rx_data)
{
	TX_Communication(nrf,tx_data);
	Select_Rx_Mode(nrf);
	HAL_Delay(50);
	RX_Communication(nrf,rx_data);
	Select_Tx_Mode(nrf);
	HAL_Delay(10);
}



void CONFIG_REG_Write(NRF_HandleTypeDef *nrf, uint8_t data)
{
	uint8_t read_reg;
	do
	{
		nRF_WriteOneRegister(nrf, CONFIG, data);
		nRF_ReadOneRegister(nrf, CONFIG, &read_reg);
	} while (read_reg != data);
}

void EN_AA_Enhanced_Shockburst_Auto_Acknowledgement_Pipe_Num(NRF_HandleTypeDef *nrf, uint8_t pipe)
{
	nRF_WriteOneRegister(nrf, EN_AA, pipe & 0x3F);
}

void RX_ADDR_P0_Write(NRF_HandleTypeDef *nrf, uint64_t data)
{
	uint8_t buff[5];
	uint64_t temp = data;
	for (int i = 0; i < 5; i++)
	{
		buff[i] = (uint8_t)((temp) & 0xff);
		temp = temp >> 8;
	}
	nRF_WriteRegister(nrf, RX_ADDR_P0, buff, 5);
}

void RX_ADDR_P1_Write(NRF_HandleTypeDef *nrf, uint64_t data)
{
	uint8_t buff[5];
	uint64_t temp = data;
	for (int i = 0; i < 5; i++)
	{
		buff[i] = (uint8_t)((temp) & 0xff);
		temp = temp >> 8;
	}
	nRF_WriteRegister(nrf, RX_ADDR_P1, buff, 5);
}

void RX_ADDR_P2_To_P5_Write(NRF_HandleTypeDef *nrf, int num, uint8_t data)
{
	uint8_t reg;
	switch (num)
	{
	case 2:
		reg = RX_ADDR_P2;
		break;
	case 3:
		reg = RX_ADDR_P3;
		break;
	case 4:
		reg = RX_ADDR_P4;
		break;
	case 5:
		reg = RX_ADDR_P5;
		break;
	default:
		return;
	}
	nRF_WriteOneRegister(nrf, reg, data);
}

void TX_ADDR_Write(NRF_HandleTypeDef *nrf, uint64_t data)
{
	uint8_t buff[5];
	uint64_t temp = data;
	for (int i = 0; i < 5; i++)
	{
		buff[i] = (uint8_t)((temp) & 0xff);
		temp = temp >> 8;
	}
	nRF_WriteRegister(nrf, TX_ADDR, buff, 5);
}

void RX_PW_P_NUM_Number_Of_Bytes(NRF_HandleTypeDef *nrf, int num, uint8_t data)
{
	uint8_t reg;
	switch (num)
	{
	case 0:
		reg = RX_PW_P0;
		break;
	case 1:
		reg = RX_PW_P1;
		break;
	case 2:
		reg = RX_PW_P2;
		break;
	case 3:
		reg = RX_PW_P3;
		break;
	case 4:
		reg = RX_PW_P4;
		break;
	case 5:
		reg = RX_PW_P5;
		break;
	default:
		return;
	}
	nRF_WriteOneRegister(nrf, reg, data);
}
