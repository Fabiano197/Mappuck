#ifndef LANDMARKS_H
#define LANDMARKS_H

#include <hal.h>


#define TOF INT16_MIN
#define IR  INT16_MAX
#define NB_WALL_LANDMARK_MAX 300
#define NB_SURFACE_LANDMARK_MAX 1000
#define NB_CORNERS_MAX 100
#define THRESHOLD_FITTING 25 //Maximal accaptable correlation error for line fitting before new corner is created [mm]
#define CLOSE_LOOP_RADIUS 100 //Radius for which wall loop will be closed

//Landmarks
typedef struct {
	int16_t x; // [mm]
	int16_t y; // [mm]
	int16_t z; // [mm]

} landmark_t;

typedef struct {
	int16_t x; // [mm]
	int16_t y; // [mm]

} wall_t;

typedef struct {
  float alpha; 	//y-axis offset [mm]
  float beta;	//slope [-]
} line_t;

/**
* @brief Enter new landmark into system, function returns true if loop is closed
*/
bool enter_landmark(landmark_t coordinates);

/**
* @brief Returns number of surface landmarks currently stored
*/
uint16_t get_nb_surface_landmarks(void);

/**
* @brief Returns pointer to first surface landmark
*/
landmark_t* get_surface_landmark_ptr(void);

/**
* @brief Returns number of wall landmarks currently stored
*/
uint16_t get_nb_wall_landmarks(void);

/**
* @brief Returns pointer to first wall landmark
*/
wall_t* get_wall_landmark_ptr(void);


/**
* @brief Returns number of corners currently stored
*/
uint16_t get_nb_corners(void);

/**
* @brief Returns pointer to first corner
*/
wall_t* get_corner_ptr(void);


#endif
