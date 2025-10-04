/*
 * rccommand.c
 *
 *  Created on: Jul 1, 2025
 *      Author: Duong Quynh
 */
#include "rccommand.h"
#include <stdio.h>
#include <string.h>

#define CENTER 2048
#define THRESHOLD 300

static RC_Command_t lastCmd = RC_CMD_NONE;
void rccommand_init(void)
{
    lastCmd = RC_CMD_NONE;
}

void rccommand_process(const RC_Input_t *input, char *buf )
{
    RC_Command_t cmd = RC_CMD_NONE;

    if ((input->throttle >= CENTER - 300 && input->throttle <= CENTER + 300) &&
        (input->yaw      >= CENTER - 300 && input->yaw      <= CENTER + 300) &&
        (input->pitch    >= CENTER - 300 && input->pitch    <= CENTER + 300) &&
        (input->roll     >= CENTER - 300 && input->roll     <= CENTER + 300))
    {
        cmd = RC_CMD_NONE;
    }

    // Throttle (UP/DOWN)
    cmd &=~((1<<0)|(1<<1));
    cmd &=~((1<<2)|(1<<3));
    if (input->throttle > CENTER + THRESHOLD)
        cmd |= RC_CMD_UP;
    else if (input->throttle < CENTER - THRESHOLD)
        cmd |= RC_CMD_DOWN;

    // Roll (LEFT/RIGHT)
    if (input->roll > CENTER + THRESHOLD)
        cmd |= RC_CMD_RIGHT;
    else if (input->roll < CENTER - THRESHOLD)
        cmd |= RC_CMD_LEFT;

    // Yaw (ROTATE)
    if (input->yaw > CENTER + THRESHOLD)
        cmd |= RC_CMD_YAW_RIGHT;
    else if (input->yaw < CENTER - THRESHOLD)
        cmd |= RC_CMD_YAW_LEFT;

    // Pitch (FORWARD/BACKWARD)
    if (input->pitch > CENTER + THRESHOLD)
        cmd |= RC_CMD_FORWARD;
    else if (input->pitch < CENTER - THRESHOLD)
    {
        cmd |= RC_CMD_BACKWARD;
    }

        buf[0] = '\0';
        //strcat(buf, "CMD:");

        if (cmd == RC_CMD_NONE)
        {
            strcat(buf, "STOP");
        }
        else
        {
            if (cmd == RC_CMD_UP)         strcat(buf, "UP|");
            if (cmd == RC_CMD_DOWN)       strcat(buf, "DOWN|");
            if (cmd == RC_CMD_LEFT)       strcat(buf, "LEFT|");
            if (cmd == RC_CMD_RIGHT)      strcat(buf, "RIGHT|");

            if (cmd == RC_CMD_FORWARD)    strcat(buf, "FW|");
            if (cmd == RC_CMD_BACKWARD)   strcat(buf, "BW|");
            if (cmd == RC_CMD_YAW_LEFT)   strcat(buf, "YL|");
            if (cmd == RC_CMD_YAW_RIGHT)  strcat(buf, "YR|");

            if (cmd & RC_CMD_UP)         strcat(buf, "UP|");
            if (cmd & RC_CMD_DOWN)       strcat(buf, "DOWN|");
            if (cmd & RC_CMD_LEFT)       strcat(buf, "LEFT|");
            if (cmd & RC_CMD_RIGHT)      strcat(buf, "RIGHT|");

            size_t len = strlen(buf);
            if (buf[len - 1] == '|') buf[len - 1] = '\0';
        }

        strcat(buf, "\n");
}


