/*
 * lcd_i2c.c
 *
 *  Created on: Jun 23, 2025
 *      Author: GIGABYTE
 */
#include "lcd_i2c.h"


/*
 * @brief This is init function, only use it in super loop
 */
void lcd_init(I2C_HandleTypeDef *hi2c)
{
	// 4 bit initialisation
	HAL_Delay(50); // wait for >40ms
	lcd_send_cmd(hi2c, 0x30);
	HAL_Delay(5); // wait for >4.1ms
	lcd_send_cmd(hi2c, 0x30);
	HAL_Delay(1); // wait for >100us
	lcd_send_cmd(hi2c, 0x30);
	HAL_Delay(10);
	lcd_send_cmd(hi2c, 0x20); // 4bit mode
	HAL_Delay(10);

	// dislay initialisation
	lcd_send_cmd(hi2c, 0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd(hi2c, 0x08); // Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd(hi2c, 0x01); // clear display
	HAL_Delay(2);
	lcd_send_cmd(hi2c, 0x06); // Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd(hi2c, 0x0C); // Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_cmd(I2C_HandleTypeDef *hi2c, char cmd)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd & 0xf0);
	data_l = ((cmd << 4) & 0xf0);
	//rs = 0 : send cmd
	data_t[0] = data_u | 0x0C; // en=1, rs=0 -> send 4 bit high, enable
	data_t[1] = data_u | 0x08; // en=0, rs=0 -> unenable
	data_t[2] = data_l | 0x0C; // en=1, rs=0 -> send 4 bit low, enable
	data_t[3] = data_l | 0x08; // en=0, rs=0 ->
	HAL_I2C_Master_Transmit(hi2c, SLAVE_ADDRESS_LCD, (uint8_t *)data_t, 4, 100);
}

void lcd_send_data(I2C_HandleTypeDef *hi2c, unsigned char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data & 0xf0);
	data_l = ((data << 4) & 0xf0);
	//rs = 1 : send data
	data_t[0] = data_u | 0x0D; // en=1, rs=1
	data_t[1] = data_u | 0x09; // en=0, rs=1
	data_t[2] = data_l | 0x0D; // en=1, rs=1
	data_t[3] = data_l | 0x09; // en=0, rs=1
	HAL_I2C_Master_Transmit(hi2c, SLAVE_ADDRESS_LCD, (uint8_t *)data_t, 4, 100);
}

void lcd_send_string(I2C_HandleTypeDef *hi2c, char line, char pos, char *str)
{
	uint16_t length = (uint16_t)strlen(str);
	lcd_send_cmd(hi2c, line + pos);

	#if USING_RTOS == 0
	HAL_Delay(1);
	#else
	vTaskDelay(pdMS_TO_TICKS(1));
	#endif

	for (int i = 0; i < length; i++)
	{
		lcd_send_data(hi2c, *(str + i));
	}
}

void lcd_turn_on_cursor(I2C_HandleTypeDef *hi2c)
{
	lcd_send_cmd(hi2c,0x0F);
}

void lcd_turn_off_cursor(I2C_HandleTypeDef *hi2c)
{
	lcd_send_cmd(hi2c,0x0C);
}

void lcd_send_next_string(I2C_HandleTypeDef *hi2c, char *str)
{
	uint16_t length = (uint16_t)strlen(str);
	for (int i = 0; i < length; i++)
	{
		lcd_send_data(hi2c, *(str + i));
	}
}

void lcd_clear(I2C_HandleTypeDef *hi2c)
{
	lcd_send_cmd(hi2c,0x01);
	for (int i = 0; i < 80; i++)
	{
		lcd_send_data(hi2c,' ');
	}
}

void lcd_startMain(I2C_HandleTypeDef *hi2c)
{
	//lcd_clear(hi2c);
	lcd_send_string(hi2c,LINE1,POS0,"MyDRONE");
	lcd_send_string(hi2c,LINE1,POS13,"Tx: ***");
	lcd_send_string(hi2c,LINE2,POS13,"Rx: ***");
	lcd_send_string(hi2c,LINE3,POS8,"Menu"); 
	lcd_send_string(hi2c,LINE4,POS8,"Settings"); 

}

void lcd_startMenu(I2C_HandleTypeDef *hi2c)
{
	//lcd_clear(hi2c);
	lcd_send_string(hi2c,LINE1,POS0,"Flight Settings");
	lcd_send_string(hi2c,LINE2,POS0,"Sensor status");
	lcd_send_string(hi2c,LINE3,POS0,"Mode");
	lcd_send_string(hi2c,LINE4,POS0,"GPS settings");
}


