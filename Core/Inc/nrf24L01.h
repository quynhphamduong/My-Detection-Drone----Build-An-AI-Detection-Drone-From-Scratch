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

#define TWO_MODOULES 0

#define CHIP_SELECT_GPIO GPIOA
#define CHIP_SELECT_PIN GPIO_PIN_4

#define CE_GPIO GPIOC
#define CE_PIN GPIO_PIN_4

#define IRQ_GPIO GPIOC
#define IRQ_PIN GPIO_PIN_5

#define NRF_SPI_TIMEOUT 100

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

void Chip_Select();
void Chip_Deselect();
void Set_CE_High();
void Set_CE_Low();
void WaitForIRQ();

void nRF_WriteRegister(SPI_HandleTypeDef *hspi,uint8_t reg,uint8_t *data,int size);
void nRF_ReadRegister(SPI_HandleTypeDef *hspi,uint8_t reg,uint8_t *receive_data,uint16_t size);
void nRF_WriteOneRegister(SPI_HandleTypeDef *hspi,uint8_t reg,uint8_t data);
void nRF_ReadOneRegister(SPI_HandleTypeDef *hspi,uint8_t reg,uint8_t *receive_data);
void nRF_SendCmd(SPI_HandleTypeDef *hspi,uint8_t cmd);
uint8_t nRF_NOPCmdGetStatus(SPI_HandleTypeDef *hspi);
void nRF_TX_Payload(SPI_HandleTypeDef *hspi,uint8_t *data, uint16_t size);
void nRF_RX_Payload(SPI_HandleTypeDef *hspi,uint8_t *rx_data, uint16_t size);

void TX_Enhanced_ShockBurst_Config(SPI_HandleTypeDef *hspi);
void RX_Enhanced_ShockBurst_Config(SPI_HandleTypeDef *hspi);

uint8_t TX_Communication(SPI_HandleTypeDef *hspi,uint8_t *data);
uint8_t RX_Communication(SPI_HandleTypeDef *hspi,uint8_t *rx_data);


void CONFIG_REG_Write(SPI_HandleTypeDef *hspi,uint8_t data);
void EN_AA_Enhanced_Shockburst_Auto_Acknowledgement_Pipe_Num(SPI_HandleTypeDef *hspi,uint8_t pipe);
void RX_ADDR_P0_Write(SPI_HandleTypeDef *hspi,uint64_t data);
void RX_ADDR_P1_Write(SPI_HandleTypeDef *hspi,uint64_t data);
void RX_ADDR_P2_To_P5_Write(SPI_HandleTypeDef *hspi,int num,uint8_t data);
void TX_ADDR_Write(SPI_HandleTypeDef *hspi,uint64_t data);
void RX_PW_P_NUM_Number_Of_Bytes(SPI_HandleTypeDef *hspi,int num,uint8_t data);



#define R_RX_PAYLOAD 0b01100001
#define W_TX_PAYLOAD 0b10100000
#define FLUSH_TX 0b11100001
#define FLUSH_RX 0b11100010
#define REUSE_TX_PL 0b11100011
#define NOP 0b11111111



#ifdef __cplusplus
}
#endif
#endif /* INC_NRF24L01_H_ */
