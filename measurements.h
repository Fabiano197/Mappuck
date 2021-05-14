#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <hal.h>

#define PROXIMITY_NORTHEAST 1
#define PROXIMITY_EAST 2

//Measurements needed for feedback
typedef struct {
	uint16_t tof_distance_front; // [mm]
	uint16_t proximity_distance_east; // [mm]
	uint16_t proximity_distance_northeast; // [mm]
	float inclination; // [rad]
} measurements_msg_t;

/**
* @brief   Initializes sensors used for feedback (time of flight, proximity and IMU)
*          Broadcasts a measurements_msg_t message on the /measurements topic
*/
void measurements_start(void);

/**
* @brief   Stops all sensors
*
*/
void measurements_stop(void);

#endif
