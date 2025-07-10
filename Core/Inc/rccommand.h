/*
 * rccommand.h
 *
 *  Created on: July 1, 2025
 *      Author: Duong Quynh
 */

#ifndef INC_RCCOMMAND_H_
#define INC_RCCOMMAND_H_
#include <stdint.h>


// Enum represents the direction of control (multiple directions can be combined using OR)
typedef enum {
    RC_CMD_NONE      = 0,
    RC_CMD_UP        = 1 << 0,
    RC_CMD_DOWN      = 1 << 1,
    RC_CMD_LEFT      = 1 << 2,
    RC_CMD_RIGHT     = 1 << 3,
    RC_CMD_FORWARD   = 1 << 4,
    RC_CMD_BACKWARD  = 1 << 5,
    RC_CMD_YAW_LEFT  = 1 << 6,
    RC_CMD_YAW_RIGHT = 1 << 7
} RC_Command_t;

// Struct stores ADC values ​​from 2 joysticks
typedef struct {
    uint16_t throttle;  // Left joystick for Y value (up/down)
    uint16_t yaw;       // Left joystick for X value (rotate left/right)
    uint16_t pitch;     // Right joystick for Y value (forward/backward)
    uint16_t roll;      // Right joystick for X value (move left/right)
} RC_Input_t;

void rccommand_init(void);

// Call this function in loop to read ADC and send command if needed
const char* rccommand_process(const RC_Input_t *input);

#endif /* INC_RCCOMMAND_H_ */
