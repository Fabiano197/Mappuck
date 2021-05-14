#include <math.h>
#include <ch.h>
#include <motors.h>

#include "mapping.h"
#include "main.h"
#include "measurements.h"
#include "landmarks.h"
#include "motor_control.h"
#include "user_feedback.h"

#define WALL_DISTANCE 50
#define EPUCK_RADIUS 35
#define WHEEL_FULLDIST_STEP 400.0
#define TICK_TO_MM 0.13
#define NB_MEASUREMENTS_INITIALIZATION 20
#define STEP_SIZE 10 // [mm]
#define MAX_USABLE_PROXIMITY_DISTANCE 85

#define STAGE_FIND_BORDER 0
#define STAGE_FOLLOW_BORDERS 1
#define STAGE_END 2

static position_t pos = {0,0,0,0,0};

static thread_t* mappingThd;
static bool mapping_configured = false;

static control_command_t u = {0, STEP_SIZE};
static control_command_t error = {0, 0};
static measurements_msg_t measurements_values;
static messagebus_topic_t *measurements_topic;

static uint8_t stage = STAGE_FIND_BORDER;

MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//Calculate control command
static void calculate_u(void){
	//All numerical values were deduced experimentally
	error.angle = (measurements_values.proximity_distance_east-WALL_DISTANCE) + 4*(measurements_values.proximity_distance_northeast-WALL_DISTANCE*3/2);
	error.angle /= 1300;
	if(measurements_values.tof_distance_front<70){
		error.angle = PI/100*(measurements_values.tof_distance_front-70);
	}
	u.angle = error.angle;
}

//Estimate current position
static void estimate_pos(void){
	static int32_t old_pos_left = 0;
	static int32_t old_pos_right = 0;
	int32_t new_pos_left  = left_motor_get_pos();
	int32_t new_pos_right = right_motor_get_pos();
	float dist_steps = TICK_TO_MM*(float)(new_pos_left + new_pos_right - old_pos_left - old_pos_right)/2;

	pos.phi -= (float)(new_pos_left-old_pos_left - new_pos_right+old_pos_right)/WHEEL_FULLDIST_STEP;
	if(pos.phi > PI)pos.phi -= 2*PI;
	else if(pos.phi <= -PI)pos.phi += 2*PI;
	pos.x += dist_steps*cos(pos.phi);
	pos.y += dist_steps*sin(pos.phi);
	pos.z += dist_steps*sin(pos.theta);
	pos.theta = measurements_values.inclination;

	old_pos_left  = new_pos_left;
	old_pos_right = new_pos_right;

	return;
}

static void set_landmarks(void){
	landmark_t l;
	if(measurements_values.proximity_distance_east <= MAX_USABLE_PROXIMITY_DISTANCE){
		l.x = pos.x + (measurements_values.proximity_distance_east+EPUCK_RADIUS)*cos(pos.phi-PI/2);
		l.y = pos.y + (measurements_values.proximity_distance_east+EPUCK_RADIUS)*sin(pos.phi-PI/2);
		l.z = IR;
		if(enter_landmark(l)){
			//Ends stage follow borders after wall loop is closed
			stage = STAGE_END;
		}
	}
	l.x = pos.x;
	l.y = pos.y;
	l.z = pos.z;
	enter_landmark(l);
}

static void find_borders(void){
	uint16_t min_dist = UINT16_MAX;
	uint16_t min_dist_index = 0;
	control_command_t rotate = {2*PI/NB_MEASUREMENTS_INITIALIZATION, 0};
	//Find shortest path to next wall
	for(uint16_t i = 0; i < NB_MEASUREMENTS_INITIALIZATION; i++){
		while(is_motor_running()) chThdSleepMilliseconds(10);
		messagebus_topic_wait(measurements_topic, &measurements_values, sizeof(measurements_values));
		make_step(rotate);
		if(measurements_values.tof_distance_front < min_dist){
			min_dist = measurements_values.tof_distance_front;
			min_dist_index = i;
		}
	}
	make_step((control_command_t){2*PI*min_dist_index/NB_MEASUREMENTS_INITIALIZATION, 0 });

	//Get close to closest wall
	while(measurements_values.tof_distance_front > WALL_DISTANCE){
		make_step(u);
		messagebus_topic_wait(measurements_topic, &measurements_values, sizeof(measurements_values));
	}
	make_step((control_command_t){-PI/2, 0 });

	while(is_motor_running()) chThdSleepMilliseconds(10);
	stage = STAGE_FOLLOW_BORDERS;
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	indicate_start_follow_walls();
}



static THD_WORKING_AREA(waMapping, 1024);
static THD_FUNCTION(mapping_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     messagebus_init(&bus, &bus_lock, &bus_condvar);

     measurements_topic = messagebus_find_topic_blocking(&bus, "/measurements");

     //This function takes 5s in order to initialize sensors and let the user time to place the robot on starting position
     indicate_startup();

	 while(chThdShouldTerminateX() == false){

		 if(stage == STAGE_FIND_BORDER){
			 find_borders();
		 }

		 if(stage == STAGE_FOLLOW_BORDERS){
			 while(is_motor_running()) chThdSleepMilliseconds(10);
			 messagebus_topic_wait(measurements_topic, &measurements_values, sizeof(measurements_values));
			 calculate_u();
			 make_step(u);
			 estimate_pos();
			 set_landmarks();
		 }
		 if(stage == STAGE_END){
			 indicate_end();
			 mapping_stop();
		 }
	 }
}

/****************************PUBLIC FUNCTIONS*************************************/


void mapping_init(void){
	if(mapping_configured)return;
	mappingThd = chThdCreateStatic(waMapping, sizeof(waMapping), NORMALPRIO+2, mapping_thd, NULL);
	mapping_configured = true;
}

void mapping_stop(void){
	if(!mapping_configured)return;
	mapping_configured = false;
	chThdTerminate(mappingThd);
}

position_t* get_position(void){
	return &pos;
}
