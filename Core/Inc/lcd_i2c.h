/*
 * lcd_i2c.h
 *
 *  Created on: Jun 23, 2025
 *      Author: GIGABYTE
 */

#ifndef INC_LCD_I2C_H_
#define INC_LCD_I2C_H_

#include "main.h"
#include "string.h"
#define SLAVE_ADDRESS_LCD 0x27<<1

#define USING_RTOS 0

enum LCD_LINE
{
	LINE1=0x80,
	LINE2=0xC0,
	LINE3=0x94,
	LINE4=0xD4
};

enum LCD_POS
{
	POS0 = 0,
	POS1 = 1,
	POS2 = 2,
	POS3 = 3,
	POS4 = 4,
	POS5 = 5,
	POS6 = 6,
	POS7 = 7,
	POS8 = 8,
	POS9 = 9,
	POS10 = 10,
	POS11 = 11,
	POS12 = 12,
	POS13 = 13,
	POS14 = 14,
	POS15 = 15,
	POS16 = 16,
	POS17 = 17,
	POS18 = 18,
	POS19 = 19,
	POS20 = 20,
};


void lcd_init (I2C_HandleTypeDef *hi2c);
void lcd_send_cmd (I2C_HandleTypeDef *hi2c,char cmd);
void lcd_send_data (I2C_HandleTypeDef *hi2c,unsigned char data);
void lcd_send_string(I2C_HandleTypeDef *hi2c, char line, char pos, char *str);
void lcd_send_next_string(I2C_HandleTypeDef *hi2c,char *str);
void lcd_clear(I2C_HandleTypeDef *hi2c);

void lcd_turn_on_cursor(I2C_HandleTypeDef *hi2c);
void lcd_turn_off_cursor(I2C_HandleTypeDef *hi2c);



#endif /* INC_LCD_I2C_H_ */
