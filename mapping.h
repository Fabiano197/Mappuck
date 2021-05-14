#ifndef MAPPING_H
#define MAPPING_H

#include <hal.h>


typedef struct {
	float x; 		//ticks
	float y; 		//ticks
	float z; 		//ticks
	float phi;   	//rad polar angle compromised between -pi and pi
	float theta; 	//rad azimuthal angle compromised between 0 and pi

} position_t;

/**
* @brief Initializes EFK
*/
void mapping_init(void);

/**
* @brief Stops EFK
*/
void mapping_stop(void);

/**
* @brief Returns estimated position of robot
*/
position_t* get_position(void);

#endif
