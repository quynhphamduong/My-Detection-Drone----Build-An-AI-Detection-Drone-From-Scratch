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

void rccommand_process(const RC_Input_t *input)
{
    RC_Command_t cmd = RC_CMD_NONE;

    // Throttle (UP/DOWN)
    if (input->throttle > CENTER + THRESHOLD)
        cmd |= RC_CMD_UP;
    else if (input->throttle < CENTER - THRESHOLD)
        cmd |= RC_CMD_DOWN;

    // Yaw (rotate)
    if (input->yaw > CENTER + THRESHOLD)
        cmd |= RC_CMD_YAW_RIGHT;
    else if (input->yaw < CENTER - THRESHOLD)
        cmd |= RC_CMD_YAW_LEFT;

    // Pitch (FORWARD/BACKWARD)
    if (input->pitch > CENTER + THRESHOLD)
        cmd |= RC_CMD_FORWARD;
    else if (input->pitch < CENTER - THRESHOLD)
        cmd |= RC_CMD_BACKWARD;

    // Roll (LEFT/RIGHT)
    if (input->roll > CENTER + THRESHOLD)
        cmd |= RC_CMD_RIGHT;
    else if (input->roll < CENTER - THRESHOLD)
        cmd |= RC_CMD_LEFT;

    if (cmd != lastCmd)
    {
        lastCmd = cmd;
        rccommand_on_command(cmd);
    }
}

void __attribute__((weak)) rccommand_on_command(RC_Command_t cmd)
{
    char buf[128] = "CMD:";

    if (cmd == RC_CMD_NONE)
    {
        strcat(buf, "STOP");
    }
    else
    {
        if (cmd & RC_CMD_UP)
            strcat(buf, "UP");
        if (cmd & RC_CMD_DOWN)
            strcat(buf, "DOWN");
        if (cmd & RC_CMD_LEFT)
            strcat(buf, "LEFT");
        if (cmd & RC_CMD_RIGHT)
            strcat(buf, "RIGHT");
        if (cmd & RC_CMD_FORWARD)
            strcat(buf, "FORWARD");
        if (cmd & RC_CMD_BACKWARD)
            strcat(buf, "BACKWARD");
        if (cmd & RC_CMD_YAW_LEFT)
            strcat(buf, "YAW_LEFT");
        if (cmd & RC_CMD_YAW_RIGHT)
            strcat(buf, "YAW_RIGHT");
    }

    strcat(buf, "\n");
}
