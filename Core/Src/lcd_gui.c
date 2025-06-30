/*
 * lcd_gui.c
 *
 *  Created on: Jun 26, 2025
 *      Author: Communist Engineers
 */

#include "lcd_gui.h"

const char *mainMenuItems[] = {"Menu", "Settings"};
const char *menuSubItems[] = {"Flight Status", "GPS status"};
const char *settingsItems[] = {"Buzzer", "GPS settings", "Mode"};
const char *gpsSettingsItem[] = {"Longtitude:", "Longtitude:"};

SimpleMenu mainMenu = {"My Drone", mainMenuItems, 2, mainMenuCallback, NULL};
SimpleMenu menuMenu = {"Menu", menuSubItems, 4, menuCallback, &mainMenu};
SimpleMenu settingsMenu = {"Settings", settingsItems, 1, settingsCallback, &mainMenu};
SimpleMenu buzzerMenu = {"Buzzer", NULL, 1, buzzerCallback, &settingsMenu};
SimpleMenu flightStatusMenu = {"Flight Status", NULL, 1, flightStatusCallback, &menuMenu};
SimpleMenu gpsStatusMenu = {"GPS Status", NULL, 1, gpsStatusCallback, &menuMenu};
SimpleMenu gpsSettingsMenu = {"GPS settings", NULL, 1, gpsSettingsCallback, &menuMenu};

SimpleMenu *currentMenu;
gpsSettingsValue gpsSettings;
uint8_t cursor = 0;
uint8_t buzzer_state = 0; // OFF

void render_menu(I2C_HandleTypeDef *hi2c)
{
    lcd_clear(hi2c);

    for (int i = 0; i < currentMenu->item_count; i++)
    {
        uint8_t line;
        if (currentMenu == &mainMenu)
        {
            if (i == 0)
            {
                line = LINE3;
            }
            else if (i == 1)
            {
                line = LINE4;
            }
        }
        else
        {
            if (i == 0)
            {
                line = LINE1;
            }
            else if (i == 1)
            {
                line = LINE2;
            }
            else if (i == 2)
            {
                line = LINE3;
            }
            else if (i == 3)
            {
                line = LINE4;
            }
        }

        if (i == cursor)
        {
            lcd_send_string(hi2c, line, POS0, ">");
        }
        else
        {
            lcd_send_string(hi2c, line, POS0, " ");
        }
        lcd_send_string(hi2c, line, POS2, (char *)currentMenu->items[i]);
    }
    if (currentMenu == &mainMenu)
    {
        lcd_send_string(hi2c, LINE1, POS0, (char *)currentMenu->title);
        lcd_send_string(hi2c, LINE1, POS12, "TX:*****");
        lcd_send_string(hi2c, LINE2, POS12, "RX:*****");
    }
}

void menu_init(I2C_HandleTypeDef *hi2c)
{
    currentMenu = &mainMenu;
    render_menu(hi2c);
}

void on_up(I2C_HandleTypeDef *hi2c)
{
    if (cursor > 0)
        cursor--;
    render_menu(hi2c);
}

void on_down(I2C_HandleTypeDef *hi2c)
{
    if (cursor < currentMenu->item_count - 1)
        cursor++;
    render_menu(hi2c);
}

void on_select(I2C_HandleTypeDef *hi2c)
{
    currentMenu->select_callback(hi2c, cursor);
}

void on_back(I2C_HandleTypeDef *hi2c)
{
    if (currentMenu->back_link != NULL)
    {
        currentMenu = currentMenu->back_link;
        cursor = 0;
        render_menu(hi2c);
    }
}

void mainMenuCallback(I2C_HandleTypeDef *hi2c, uint8_t index)
{
    if (index == 0)
    {
        currentMenu = &menuMenu;
        cursor = 0;
    }
    else if (index == 1)
    {
        currentMenu = &settingsMenu;
        cursor = 0;
    }
    render_menu(hi2c);
}

void menuCallback(I2C_HandleTypeDef *hi2c, uint8_t index)
{
    if (index == 0)
    {
        currentMenu = &flightStatusMenu;
        currentMenu->select_callback(hi2c, index);
    }
    else if (index == 1)
    {
        currentMenu = &settingsMenu;
        currentMenu->select_callback(hi2c, index);
    }
}

void settingsCallback(I2C_HandleTypeDef *hi2c, uint8_t index)
{
    if (index == 0)
    {
        currentMenu = &buzzerMenu;
        cursor = 0;
        if (buzzer_state == 0)
        {
            lcd_clear(hi2c);
            lcd_send_string(hi2c, LINE1, POS0, "OFF");
        }
        else
        {
            lcd_clear(hi2c);
            lcd_send_string(hi2c, LINE1, POS0, "ON");
        }
    }
    else if (index == 1)
    {
        currentMenu = &gpsSettingsMenu;
        cursor = 0;
        render_menu(hi2c);
    }
    else if (index==2)
    {

    }
}


void buzzerCallback(I2C_HandleTypeDef *hi2c, uint8_t index)
{
    buzzer_state = buzzer_state ^ 1;
    if (buzzer_state == 0)
    {
        lcd_clear(hi2c);
        lcd_send_string(hi2c, LINE1, POS0, "OFF");
    }
    else
    {
        lcd_clear(hi2c);
        lcd_send_string(hi2c, LINE1, POS0, "ON");
    }
}

void flightStatusCallback(I2C_HandleTypeDef *hi2c, uint8_t index)
{
    lcd_clear(hi2c);
    lcd_send_string(hi2c, LINE1, POS0, "Roll:");
    lcd_send_string(hi2c, LINE2, POS0, "Pitch:");
    lcd_send_string(hi2c, LINE3, POS0, "Yaw:");
    lcd_send_string(hi2c, LINE3, POS0, "Altitude:");
}

void gpsStatusCallback(I2C_HandleTypeDef *hi2c, uint8_t index)
{
    lcd_clear(hi2c);
    lcd_send_string(hi2c, LINE1, POS0, "Longtitude:");
    lcd_send_string(hi2c, LINE2, POS0, "Latitude:");
}

void gpsSettingsCallback(I2C_HandleTypeDef *hi2c, uint8_t index)
{
    lcd_clear(hi2c);
    lcd_send_string(hi2c, LINE1, POS0, "Longtitude:");
    lcd_send_string(hi2c, LINE2, POS0, "Latitude:");
}
