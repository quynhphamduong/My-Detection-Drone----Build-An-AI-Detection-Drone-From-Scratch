
/*
 * nRF24L01.h
 *
 *  Created on: Jan 24, 2025
 *      Author: GIGABYTE
 */

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"

#define USING_NRF_RTOS 1

#if USING_NRF_RTOS==1
#include "cmsis_os.h"
#endif

//#define CHIP_SELECT_GPIO GPIOA
//#define CHIP_SELECT_PIN GPIO_PIN_4

//#define CE_GPIO GPIOB
//#define CE_PIN GPIO_PIN_0
//
//#define IRQ_GPIO GPIOB
//#define IRQ_PIN GPIO_PIN_1

#define NRF_SPI_TIMEOUT 100

typedef struct
{
	GPIO_TypeDef *CS_GPIO;
	GPIO_TypeDef *CE_GPIO;
	GPIO_TypeDef *IRQ_GPIO;
	uint16_t CS_PIN;
	uint16_t CE_PIN;
	uint16_t IRQ_PIN;
	SPI_HandleTypeDef *hspi;
	uint8_t nrfmode;
}NRF_HandleTypeDef;

typedef enum
{
	 CONFIG,
	 EN_AA,//Enable shockburst
	 EN_RXADDR,//Enabled RX Addresses
	 SETUP_AW,//set up address width
	 SETUP_RETR,//Setup of Automatic Retransmission
	 RF_CH,//
	 RF_SETUP,//
	 STATUS,//Status Register (In parallel to the SPI instruction word applied on the MOSI pin, the STATUS register is shifted serially out on the MISO pin)
	 OBSERVE_TX,//Transmit observe register
	 CD,//
	 RX_ADDR_P0,
	 RX_ADDR_P1,
	 RX_ADDR_P2,
	 RX_ADDR_P3,
	 RX_ADDR_P4,
	 RX_ADDR_P5,
	//Receive address data pipe 0. 5 Bytes maximum length. (LSByte is written first.Write the number of bytes defined by SETUP_AW)
	//Receive address data pipe 1. 5 Bytes maximum length. (LSByte is written first.Write the number of bytes defined by SETUP_AW)
	//Receive address data pipe 2. Only LSB.MSBytes will be equal to RX_ADDR_P1[39:8]
	//Receive address data pipe 3. Only LSB.MSBytes will be equal to RX_ADDR_P1[39:8]
	//Receive address data pipe 4. Only LSB.MSBytes will be equal to RX_ADDR_P1[39:8]
	//Receive address data pipe 5. Only LSB.MSBytes will be equal to RX_ADDR_P1[39:8]
	 TX_ADDR,
	//Transmit address. Used for a PTX device only. (LSByte is written first) Set RX_ADDR_P0 equal to this address to handle automatic acknowledge if this is a PTX device with Enhanced ShockBurstâ„¢ enabled
	 RX_PW_P0,
	 RX_PW_P1,
	 RX_PW_P2,
	 RX_PW_P3,
	 RX_PW_P4,
	 RX_PW_P5,
	//Number of bytes in RX payload in data pipe 0 (1 to 32 bytes). 0 Pipe not used 1 = 1 byteâ€¦ 32 = 32 bytes (5:0)
	//Number of bytes in RX payload in data pipe 1 (1 to 32 bytes). 0 Pipe not used 1 = 1 byteâ€¦ 32 = 32 bytes (5:0)
	//Number of bytes in RX payload in data pipe 2 (1 to 32 bytes). 0 Pipe not used 1 = 1 byteâ€¦ 32 = 32 bytes (5:0)
	//Number of bytes in RX payload in data pipe 3 (1 to 32 bytes). 0 Pipe not used 1 = 1 byteâ€¦ 32 = 32 bytes (5:0)
	//Number of bytes in RX payload in data pipe 4 (1 to 32 bytes). 0 Pipe not used 1 = 1 byteâ€¦ 32 = 32 bytes (5:0)
	//Number of bytes in RX payload in data pipe 5 (1 to 32 bytes). 0 Pipe not used 1 = 1 byteâ€¦ 32 = 32 bytes (5:0)
	 FIFO_STATUS,

}NRF24L01_REGISTER;

void Chip_Select(NRF_HandleTypeDef *nrf);
void Chip_Deselect(NRF_HandleTypeDef *nrf);
void Set_CE_High(NRF_HandleTypeDef *nrf);
void Set_CE_Low(NRF_HandleTypeDef *nrf);
void WaitForIRQ(NRF_HandleTypeDef *nrf);

void nRF_WriteRegister(NRF_HandleTypeDef *nrf, uint8_t reg, uint8_t *data, int size);
void nRF_WriteOneRegister(NRF_HandleTypeDef *nrf, uint8_t reg, uint8_t data);
void nRF_ReadRegister(NRF_HandleTypeDef *nrf, uint8_t reg, uint8_t *receive_data, uint16_t size);
void nRF_ReadOneRegister(NRF_HandleTypeDef *nrf, uint8_t reg, uint8_t *receive_data);
void nRF_SendCmd(NRF_HandleTypeDef *nrf, uint8_t cmd);
uint8_t nRF_GetStatus(NRF_HandleTypeDef *nrf);
void nRF_TX_Payload(NRF_HandleTypeDef *nrf, uint8_t *data, uint16_t size);
void nRF_RX_Payload(NRF_HandleTypeDef *nrf, uint8_t *rx_data, uint16_t size);

void TX_Enhanced_ShockBurst_Config(NRF_HandleTypeDef *nrf,uint64_t tx_addr);
void RX_Enhanced_ShockBurst_Config(NRF_HandleTypeDef *nrf,uint64_t rx_addr);

void Select_Tx_Mode(NRF_HandleTypeDef *nrf);
void Select_Rx_Mode(NRF_HandleTypeDef *nrf);

#if USING_NRF_RTOS==1
void TX_Enhanced_ShockBurst_Config_RTOS(NRF_HandleTypeDef *nrf);
void Select_Tx_Mode_RTOS(NRF_HandleTypeDef *nrf);
void RX_Enhanced_ShockBurst_Config_RTOS(NRF_HandleTypeDef *nrf);
void Select_Rx_Mode_RTOS(NRF_HandleTypeDef *nrf);
void Two_Way_Commuination_RTOS(NRF_HandleTypeDef *nrf,uint8_t *tx_data,uint8_t *rx_data);
#endif


void Two_Way_Commuination_Pipe0_Config(NRF_HandleTypeDef *nrf, uint64_t tx_addr, uint64_t rx_addr);
void Two_Way_Commuination_Pipe1_Config(NRF_HandleTypeDef *nrf, uint64_t tx_addr, uint64_t rx_addr);

uint8_t TX_Communication(NRF_HandleTypeDef *nrf, uint8_t *data);
uint8_t RX_Communication(NRF_HandleTypeDef *nrf, uint8_t *rx_data);
void Two_Way_Commuination(NRF_HandleTypeDef *nrf,uint8_t *tx_data,uint8_t *rx_data);



void CONFIG_REG_Write(NRF_HandleTypeDef *nrf, uint8_t data);
void EN_AA_Enhanced_Shockburst_Auto_Acknowledgement_Pipe_Num(NRF_HandleTypeDef *nrf, uint8_t pipe);
void RX_ADDR_P0_Write(NRF_HandleTypeDef *nrf, uint64_t data);
void RX_ADDR_P1_Write(NRF_HandleTypeDef *nrf, uint64_t data);
void RX_ADDR_P2_To_P5_Write(NRF_HandleTypeDef *nrf, int num, uint8_t data);
void TX_ADDR_Write(NRF_HandleTypeDef *nrf, uint64_t data);
void RX_PW_P_NUM_Number_Of_Bytes(NRF_HandleTypeDef *nrf, int num, uint8_t data);



#define R_RX_PAYLOAD 0b01100001
#define W_TX_PAYLOAD 0b10100000
#define FLUSH_TX 0b11100001
#define FLUSH_RX 0b11100010
#define REUSE_TX_PL 0b11100011
#define NOP 0b11111111

typedef enum
{
	PRIM_RX,
	PWR_UP,
	CRCO,
	EN_CRC,
	MASK_MAX_RT,
	MASK_TX_DS,
	MASK_RX_DR,
}CONFIG_BIT;

typedef enum
{
	TX_FULL,
	RX_P_NO,
	RX_P_NO1,
	RX_P_NO2,
	MAX_RT,
	TX_DS,
	RX_DR,
}STATUS_BIT;

enum
{
	STATUS_TX_NONDEFINE,
	STATUS_TX_ERROR,
	STATUS_TX_OK,
	STATUS_RX_NONEDEFINE=0,
	STATUS_RX_ERROR,
	STATUS_RX_OK,
};

typedef enum
{
	FIFO_RX_EMPTY,
	FIFO_RX_FULL,
	FIFO_TX_EMPTY=4,
	FIFO_TX_FULL,
	FIFO_TX_REUSE
}FIFO_STATUS_BIT;

typedef enum
{
	MODE_TX,
	MODE_RX
}NRF_MODE;

//extern NRF_MODE nrfmode;

#ifdef __cplusplus
}
#endif
#endif /* INC_NRF24L01_H_ */
