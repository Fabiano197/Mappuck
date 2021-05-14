#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H

#include <hal.h>

/**
* @brief Starts Bluetooth communication with computer
*/
void communications_init(void);

/**
* @brief Stops Bluetooth communication with computer
*/
void communications_stop(void);

#endif
