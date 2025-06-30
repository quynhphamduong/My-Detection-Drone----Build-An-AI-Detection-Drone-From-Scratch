/*
 * nRF24L01.c
 *
 *  Created on: Jan 24, 2025
 *      Author: GIGABYTE
 */

#include "nRF24L01.h"
#include "main.h"

NRF_MODE nrfmode;

void Chip_Select()
{
	HAL_GPIO_WritePin(CHIP_SELECT_GPIO, CHIP_SELECT_PIN, 0);
}

void Chip_Deselect()
{
	HAL_GPIO_WritePin(CHIP_SELECT_GPIO, CHIP_SELECT_PIN, 1);
}

void Set_CE_High()
{
	HAL_GPIO_WritePin(CE_GPIO, CE_PIN, 1);
}

void Set_CE_Low()
{
	HAL_GPIO_WritePin(CE_GPIO, CE_PIN, 0);
}

void WaitForIRQ()
{
	while (HAL_GPIO_ReadPin(IRQ_GPIO, IRQ_PIN) == 1)
		;
}

void nRF_WriteRegister(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data, int size)
{
	uint8_t buff[size + 1];
	buff[0] = reg | (1 << 5);
	for (int i = 0; i < size; i++)
	{
		buff[i + 1] = data[i];
	}
	Chip_Select();
	HAL_SPI_Transmit(hspi, buff, (uint16_t)size + 1, NRF_SPI_TIMEOUT);
	Chip_Deselect();
}

void nRF_WriteOneRegister(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t data)
{
	uint8_t buff[2];
	buff[0] = reg | (1 << 5);
	buff[1] = data;
	Chip_Select();
	HAL_SPI_Transmit(hspi, buff, 2, NRF_SPI_TIMEOUT);
	Chip_Deselect();
}

void nRF_ReadRegister(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *receive_data, uint16_t size)
{

	uint8_t buff = reg;
	Chip_Select();
	HAL_SPI_Transmit(hspi, &buff, 1, NRF_SPI_TIMEOUT);
	HAL_SPI_Receive(hspi, receive_data, size, NRF_SPI_TIMEOUT * 10);
	Chip_Deselect();
}

void nRF_ReadOneRegister(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *receive_data)
{

	uint8_t buff = reg;
	Chip_Select();
	HAL_SPI_Transmit(hspi, &buff, 1, NRF_SPI_TIMEOUT);
	HAL_SPI_Receive(hspi, receive_data, 1, NRF_SPI_TIMEOUT);
	Chip_Deselect();
}
void nRF_SendCmd(SPI_HandleTypeDef *hspi, uint8_t cmd)
{

	uint8_t buff = cmd;
	Chip_Select();
	HAL_SPI_Transmit(hspi, &buff, 1, NRF_SPI_TIMEOUT);
	Chip_Deselect();
}

uint8_t nRF_GetStatus(SPI_HandleTypeDef *hspi)
{

	uint8_t buff = STATUS;
	uint8_t rx_data = 0;
	Chip_Select();
	HAL_SPI_Transmit(hspi, &buff, 1, NRF_SPI_TIMEOUT);
	HAL_SPI_Receive(hspi, &rx_data, 1, NRF_SPI_TIMEOUT * 10);
	Chip_Deselect();
	return rx_data;
}

void nRF_TX_Payload(SPI_HandleTypeDef *hspi, uint8_t *data, uint16_t size)
{
	Chip_Select();
	uint8_t buff = W_TX_PAYLOAD;
	HAL_SPI_Transmit(hspi, &buff, 1, NRF_SPI_TIMEOUT);
	HAL_SPI_Transmit(hspi, data, size, NRF_SPI_TIMEOUT);
	Chip_Deselect();
}

void nRF_RX_Payload(SPI_HandleTypeDef *hspi, uint8_t *rx_data, uint16_t size)
{
	Chip_Select();
	uint8_t buff = R_RX_PAYLOAD;
	HAL_SPI_Transmit(hspi, &buff, 1, NRF_SPI_TIMEOUT);
	HAL_SPI_Receive(hspi, rx_data, size, NRF_SPI_TIMEOUT * 10);
	Chip_Deselect();
}

void TX_Enhanced_ShockBurst_Config(SPI_HandleTypeDef *hspi)
{
	uint8_t buff = 0x0a;
	Set_CE_Low();
	CONFIG_REG_Write(hspi, buff);
	nRF_WriteOneRegister(hspi, EN_AA, 0x00);
	nRF_WriteOneRegister(hspi, RF_SETUP, 0x7);
	Set_CE_High();
	HAL_Delay(2);
	nrfmode = MODE_TX;
}

void Select_Tx_Mode(SPI_HandleTypeDef *hspi)
{
	uint8_t buff = 0x0a;
	Set_CE_Low();
	CONFIG_REG_Write(hspi, buff);
	Set_CE_High();
	HAL_Delay(2);
	nrfmode = MODE_TX;
}

void RX_Enhanced_ShockBurst_Config(SPI_HandleTypeDef *hspi)
{
	uint8_t buff = 0xb;
	//	Set_CE_Low();
	CONFIG_REG_Write(hspi, buff);
	nRF_WriteOneRegister(hspi, EN_RXADDR, 0x01);
	nRF_WriteOneRegister(hspi, RX_PW_P0, 8);
	nRF_WriteOneRegister(hspi, EN_AA, 0x00);
	nRF_WriteOneRegister(hspi, RF_SETUP, 0x7);
	Set_CE_High();
	HAL_Delay(2);
	nRF_SendCmd(hspi, FLUSH_RX);
	nrfmode = MODE_RX;
}

void Select_Rx_Mode(SPI_HandleTypeDef *hspi)
{
	uint8_t buff = 0x0b;
	Set_CE_Low();
	CONFIG_REG_Write(hspi, buff);
	Set_CE_High();
	HAL_Delay(2);
	nRF_SendCmd(hspi, FLUSH_RX);
	nrfmode = MODE_RX;
}

#if USING_NRF_RTOS==1
void TX_Enhanced_ShockBurst_Config_RTOS(SPI_HandleTypeDef *hspi)
{
	uint8_t buff = 0x0a;
	Set_CE_Low();
	CONFIG_REG_Write(hspi, buff);
	nRF_WriteOneRegister(hspi, EN_AA, 0x00);
	nRF_WriteOneRegister(hspi, RF_SETUP, 0x7);
	Set_CE_High();
	vTaskDelay(pdMS_TO_TICKS(2));
	nrfmode = MODE_TX;
}

void Select_Tx_Mode_RTOS(SPI_HandleTypeDef *hspi)
{
	uint8_t buff = 0x0a;
	Set_CE_Low();
	CONFIG_REG_Write(hspi, buff);
	Set_CE_High();
	vTaskDelay(pdMS_TO_TICKS(2));
	nrfmode = MODE_TX;
}

void RX_Enhanced_ShockBurst_Config_RTOS(SPI_HandleTypeDef *hspi)
{
	uint8_t buff = 0xb;
	Set_CE_Low();
	CONFIG_REG_Write(hspi, buff);
	nRF_WriteOneRegister(hspi, EN_RXADDR, 0x01);
	nRF_WriteOneRegister(hspi, RX_PW_P0, 8);
	nRF_WriteOneRegister(hspi, EN_AA, 0x00);
	nRF_WriteOneRegister(hspi, RF_SETUP, 0x7);
	Set_CE_High();
	vTaskDelay(pdMS_TO_TICKS(2));
	nRF_SendCmd(hspi, FLUSH_RX);
	nrfmode = MODE_RX;
}

void Select_Rx_Mode_RTOS(SPI_HandleTypeDef *hspi)
{
	uint8_t buff = 0x0b;
	Set_CE_Low();
	CONFIG_REG_Write(hspi, buff);
	Set_CE_High();
	vTaskDelay(pdMS_TO_TICKS(2));
	nrfmode = MODE_RX;
}
#endif

void Two_Way_Commuination_Pipe0_Config(SPI_HandleTypeDef *hspi, uint64_t tx_addr, uint64_t rx_addr)
{
	Set_CE_Low();
	RX_PW_P_NUM_Number_Of_Bytes(hspi, 0, 8);
	TX_ADDR_Write(hspi, tx_addr);
	RX_ADDR_P0_Write(hspi, rx_addr);
	nRF_WriteOneRegister(hspi, EN_RXADDR, 1);
	nRF_WriteOneRegister(hspi, EN_AA, 0x00);
	nRF_WriteOneRegister(hspi, RF_SETUP, 0x7);
	Set_CE_High();
	HAL_Delay(2);
}

void Two_Way_Commuination_Pipe1_Config(SPI_HandleTypeDef *hspi, uint64_t tx_addr, uint64_t rx_addr)
{
	Set_CE_Low();
	RX_PW_P_NUM_Number_Of_Bytes(hspi, 1, 8);
	RX_ADDR_P1_Write(hspi, rx_addr);
	TX_ADDR_Write(hspi, tx_addr);
	nRF_WriteOneRegister(hspi, EN_RXADDR, 2);
	nRF_WriteOneRegister(hspi, EN_AA, 0x00);
	nRF_WriteOneRegister(hspi, RF_SETUP, 0x7);
	Set_CE_High();
	HAL_Delay(2);
}

uint8_t TX_Communication(SPI_HandleTypeDef *hspi, uint8_t *data)
{
	if (nrfmode == MODE_TX)
	{
		nRF_TX_Payload(hspi, data, 8);
		Set_CE_High();
		WaitForIRQ();
		nRF_SendCmd(hspi, FLUSH_TX);
		uint8_t status = nRF_GetStatus(hspi);
		if ((status & (1 << MAX_RT)) != 0)
		{
			status |= ((1 << MAX_RT) | (1 << TX_FULL));
			nRF_WriteOneRegister(hspi, STATUS, status);
			return STATUS_TX_ERROR;
		}
		else if ((status & (1 << 5)) != 0)
		{
			status |= ((1 << TX_DS) | (1 << TX_FULL));
			nRF_WriteOneRegister(hspi, STATUS, status);
			return STATUS_TX_OK;
		}
	}
	return STATUS_TX_NONDEFINE;
}

uint8_t RX_Communication(SPI_HandleTypeDef *hspi, uint8_t *rx_data)
{
	if (nrfmode == MODE_RX)
	{
		Set_CE_High();
		uint8_t status = nRF_GetStatus(hspi);
		if ((status & (1 << RX_DR)) != 0)
		{
			nRF_WriteOneRegister(hspi, STATUS, (1 << 6));
			nRF_RX_Payload(hspi, rx_data, 8);
			return STATUS_RX_OK;
		}
		nRF_SendCmd(hspi, FLUSH_RX);
		return STATUS_RX_ERROR;
	}
	return STATUS_RX_NONEDEFINE;
}

void Two_Way_Commuination(SPI_HandleTypeDef *hspi,uint8_t *tx_data,uint8_t *rx_data)
{
	TX_Communication(hspi,tx_data);
	Select_Rx_Mode_RTOS(hspi);
	HAL_Delay(50);
	RX_Communication(hspi,rx_data);
	Select_Tx_Mode_RTOS(hspi);
	HAL_Delay(10);
}

void Two_Way_Commuination_RTOS(SPI_HandleTypeDef *hspi,uint8_t *tx_data,uint8_t *rx_data)
{
	TX_Communication(hspi,tx_data);
	Select_Rx_Mode_RTOS(hspi);
	vTaskDelay(pdMS_TO_TICKS(50));
	RX_Communication(hspi,rx_data);
	Select_Tx_Mode_RTOS(hspi);
	vTaskDelay(pdMS_TO_TICKS(10));
}

void CONFIG_REG_Write(SPI_HandleTypeDef *hspi, uint8_t data)
{
	uint8_t read_reg;
	do
	{
		nRF_WriteOneRegister(hspi, CONFIG, data);
		nRF_ReadOneRegister(hspi, CONFIG, &read_reg);
	} while (read_reg != data);
}

void EN_AA_Enhanced_Shockburst_Auto_Acknowledgement_Pipe_Num(SPI_HandleTypeDef *hspi, uint8_t pipe)
{
	nRF_WriteOneRegister(hspi, EN_AA, pipe & 0x3F);
}

void RX_ADDR_P0_Write(SPI_HandleTypeDef *hspi, uint64_t data)
{
	uint8_t buff[5];
	uint64_t temp = data;
	for (int i = 0; i < 5; i++)
	{
		buff[i] = (uint8_t)((temp) & 0xff);
		temp = temp >> 8;
	}
	nRF_WriteRegister(hspi, RX_ADDR_P0, buff, 5);
}

void RX_ADDR_P1_Write(SPI_HandleTypeDef *hspi, uint64_t data)
{
	uint8_t buff[5];
	uint64_t temp = data;
	for (int i = 0; i < 5; i++)
	{
		buff[i] = (uint8_t)((temp) & 0xff);
		temp = temp >> 8;
	}
	nRF_WriteRegister(hspi, RX_ADDR_P1, buff, 5);
}

void RX_ADDR_P2_To_P5_Write(SPI_HandleTypeDef *hspi, int num, uint8_t data)
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
	nRF_WriteOneRegister(hspi, reg, data);
}

void TX_ADDR_Write(SPI_HandleTypeDef *hspi, uint64_t data)
{
	uint8_t buff[5];
	uint64_t temp = data;
	for (int i = 0; i < 5; i++)
	{
		buff[i] = (uint8_t)((temp) & 0xff);
		temp = temp >> 8;
	}
	nRF_WriteRegister(hspi, TX_ADDR, buff, 5);
}

void RX_PW_P_NUM_Number_Of_Bytes(SPI_HandleTypeDef *hspi, int num, uint8_t data)
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
	nRF_WriteOneRegister(hspi, reg, data);
}
