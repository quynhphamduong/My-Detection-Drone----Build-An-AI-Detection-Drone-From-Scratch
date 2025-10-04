/*
 * lcd_gui.h
 *
 *  Created on: Jun 26, 2025
 *      Author: PC
 */

#ifndef INC_LCD_GUI_H_
#define INC_LCD_GUI_H_
#include "lcd_i2c.h"

typedef struct SimpleMenu
{
    const char *title;
    const char **items;
    uint8_t item_count;
    void (*select_callback)(I2C_HandleTypeDef *hi2c, uint8_t index);
    struct SimpleMenu *back_link;
} SimpleMenu;

typedef struct
{
    float Longtitude;
    float Lattitude;
} gpsSettingsValue;

extern SimpleMenu mainMenu;
extern SimpleMenu menuMenu;
extern SimpleMenu settingsMenu;
extern SimpleMenu buzzerMenu;

extern gpsSettingsValue gpsSetting;

extern const char *mainMenuItems[];
extern const char *menuSubItems[];
extern const char *settingsItems[];

extern SimpleMenu *currentMenu;
extern uint8_t cursor;
extern uint8_t buzzer_state;

void render_menu(I2C_HandleTypeDef *hi2c);
void mainMenuCallback(I2C_HandleTypeDef *hi2c, uint8_t index);
void menuCallback(I2C_HandleTypeDef *hi2c, uint8_t index);
void settingsCallback(I2C_HandleTypeDef *hi2c, uint8_t index);
void buzzerCallback(I2C_HandleTypeDef *hi2c, uint8_t index);
void flightStatusCallback(I2C_HandleTypeDef *hi2c, uint8_t index);
void gpsStatusCallback(I2C_HandleTypeDef *hi2c, uint8_t index);
void gpsSettingsCallback(I2C_HandleTypeDef *hi2c, uint8_t index);
void on_up(I2C_HandleTypeDef *hi2c);
void on_down(I2C_HandleTypeDef *hi2c);
void on_select(I2C_HandleTypeDef *hi2c);
void on_back(I2C_HandleTypeDef *hi2c);
void menu_init(I2C_HandleTypeDef *hi2c);

#endif /* INC_LCD_GUI_H_ */
