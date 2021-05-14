#ifndef USER_FEEDBACK_H
#define USER_FEEDBACK_H

#include <hal.h>

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} rgb_colour_t;

/**
* @brief Starts user feedback module
*/
void user_feedback_init(void);

/**
* @brief Signals startup of robot by means of leds and sound.
* This function takes about 5s in order to let the other modules and sensors start up correctly and give the user time to place the robot.
*/
void indicate_startup(void);

/**
* @brief Signals start of follow walls
*/
void indicate_start_follow_walls(void);

/**
* @brief Signals that wall measuring is completed
*/
void indicate_end(void);

#endif
