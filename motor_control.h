#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <hal.h>

#define MOTORSPEED 200 //[ticks/s] which corresponds to 0.13*MOTORSPEED [mm/s]

typedef struct {
	float angle;
	uint16_t dist;
} control_command_t;

/**
* @brief Init control
*/
void motor_control_init(void);

/**
* @brief Stops control
*/
void motor_control_stop(void);

/**
* @brief Turns angle radians and moves distance mm
*/
void make_step(control_command_t u);

/**
* @brief Returns whether motors are running
*/
bool is_motor_running(void);

#endif
