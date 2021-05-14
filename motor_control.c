#include <motors.h>

#include "motor_control.h"
#include "main.h"
#include "math.h"
#include "measurements.h"

#define ANGLE_TO_TIME 208100
#define DISTANCE_TO_TIME 7950

static thread_t *motorControlThd;
static bool motor_control_configured = false;

static control_command_t target_u;
static bool motor_running = false;

static binary_semaphore_t job_available_bsem;
static binary_semaphore_t motor_running_bsem;

static THD_WORKING_AREA(waMotorControl, 512);

static THD_FUNCTION(motor_control_thd, arg) {
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	while(chThdShouldTerminateX() == false){
		chBSemWait(&job_available_bsem);
		motor_running = true;
		// turn by the angle u.angle in rad
		if(target_u.angle < 0){
			right_motor_set_speed(MOTORSPEED);
			left_motor_set_speed(-MOTORSPEED);
			chThdSleepMilliseconds((uint16_t)(-target_u.angle*ANGLE_TO_TIME/MOTORSPEED+1));
		}
		else if(target_u.angle > 0){
			right_motor_set_speed(-MOTORSPEED);
			left_motor_set_speed(MOTORSPEED);
			chThdSleepMilliseconds((uint16_t)(target_u.angle*ANGLE_TO_TIME/MOTORSPEED+1));
		}

		// advance u.dist in mm
		if(target_u.dist > 0){
			right_motor_set_speed(MOTORSPEED);
			left_motor_set_speed(MOTORSPEED);
			chThdSleepMilliseconds(target_u.dist*DISTANCE_TO_TIME/MOTORSPEED+1);
		}

		right_motor_set_speed(0);
		left_motor_set_speed(0);
		motor_running = false;
		chBSemSignal(&motor_running_bsem);
     }
}


/****************************PUBLIC FUNCTIONS*************************************/

void motor_control_init(void){
	if(motor_control_configured)return;
	motors_init();
	chBSemObjectInit(&motor_running_bsem, false);
	chBSemObjectInit(&job_available_bsem, true);
	motorControlThd = chThdCreateStatic(waMotorControl, sizeof(waMotorControl), NORMALPRIO+3, motor_control_thd, NULL);
	motor_control_configured = true;
}

void motor_control_stop(void){
	if(!motor_control_configured)return;
	motor_control_configured = false;
	chThdTerminate(motorControlThd);
}

void make_step(control_command_t u){
	//Assures that control command is not changed while motors are executing previous job
	chBSemWait(&motor_running_bsem);
	target_u = u;
	//Tells the motor thread that a new job is available
	chBSemSignal(&job_available_bsem);
}

bool is_motor_running(void){
	return motor_running;
}
