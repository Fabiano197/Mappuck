#include <ch.h>
#include <chprintf.h>

#include "communications.h"
#include "main.h"
#include "mapping.h"
#include "landmarks.h"


static thread_t *comThd;
static bool com_configured = false;

static void send_data_Bluetooth(void){
	//Send start message for identification of communication start
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);

	//Send current robot position
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)get_position(), sizeof(position_t));

	//Send corners
	uint16_t nb_corners = get_nb_corners();
	wall_t* corner_ptr = get_corner_ptr();
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&nb_corners, sizeof(uint16_t));
	for(uint16_t i = 0; i < nb_corners; i++){
		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)(corner_ptr+i), sizeof(wall_t));
	}

	//Send wall landmarks
	uint16_t nb_wall_landmarks = get_nb_wall_landmarks();
	wall_t* wall_landmark_ptr = get_wall_landmark_ptr();
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&nb_wall_landmarks, sizeof(uint16_t));
	for(uint16_t i = 0; i < nb_wall_landmarks; i++){
		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)(wall_landmark_ptr+i), sizeof(wall_t));
	}

	//Send surface landmarks (only the ones which hasn't been sent yet because this number can get quite big)
	static uint16_t nb_surface_landmarks_already_sent = 0;
	uint16_t nb_surface_landmarks = get_nb_surface_landmarks();
	uint16_t nb_surface_landmarks_to_send = nb_surface_landmarks-nb_surface_landmarks_already_sent;
	landmark_t* surface_landmark_ptr = get_surface_landmark_ptr();
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&nb_surface_landmarks_to_send, sizeof(uint16_t));
	for(uint16_t i = nb_surface_landmarks_already_sent; i < nb_surface_landmarks; i++){
		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)(surface_landmark_ptr+i), sizeof(landmark_t));
	}
	nb_surface_landmarks_already_sent = nb_surface_landmarks;

	return;
}

static THD_WORKING_AREA(waCommunication, 512);

static THD_FUNCTION(communication_thd, arg) {
	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	//Give system time to start up
	chThdSleepMilliseconds(100);

	//Send reset message to reset plot on computer
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"RESET", 5);

	while(chThdShouldTerminateX() == false){
		send_data_Bluetooth();
		chThdSleepMilliseconds(1000);
	}
}

/****************************PUBLIC FUNCTIONS*************************************/

void communications_init(void){
	if(com_configured)return;
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};
	sdStart(&SD3, &ser_cfg); // UART3.

	comThd = chThdCreateStatic(waCommunication, sizeof(waCommunication), NORMALPRIO+1, communication_thd, NULL);
	com_configured = true;
}

void communication_stop(void){
	if(!com_configured)return;
	com_configured = false;
	chThdTerminate(comThd);
}
