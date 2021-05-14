#include <ch.h>
#include <audio/play_melody.h>
#include <audio/audio_thread.h>
#include <leds.h>
#include <spi_comm.h>

#include "user_feedback.h"
#include "main.h"

static const rgb_colour_t colours[8] = {
	{
		100, 0, 0 //RED
	},
	{
		100, 25, 0 //ORANGE
	},
	{
		100, 85, 0 //YELLOW
	},
	{
		0, 100, 0 //GREEN
	},
	{
		0, 100, 20 //TURQUOISE
	},
	{
		0, 0, 100 //BLUE
	},
	{
		90, 0, 20 //PINK
	},
	{
		100, 100, 100 //WHITE
	}
};

typedef enum{
	RED = 0,
	ORGANGE,
	YELLOW,
	GREEN,
	TURQUOISE,
	BLUE,
	PINK,
	WHITE,
	NB_COLOURS
} colour_t;

/****************************PUBLIC FUNCTIONS*************************************/

void user_feedback_init(void){
	dac_start();
	spi_comm_start();
	playMelodyStart();
}

void indicate_startup(void){
	//Note: In order to synchronize the leds and melody, instead of playMelody, playNote is used here

	chThdSleepMilliseconds(500);

	//Sec 1
	set_led(LED1, 1);
	playNote(NOTE_C4, 200);
	chThdSleepMilliseconds(300);
	set_rgb_led(LED2, colours[RED].r, colours[RED].g, colours[RED].b);
	chThdSleepMilliseconds(500);

	//Sec 2
	set_led(LED3, 1);
	playNote(NOTE_C4, 200);
	chThdSleepMilliseconds(300);
	set_rgb_led(LED4, colours[RED].r, colours[RED].g, colours[RED].b);
	chThdSleepMilliseconds(500);

	//Sec 3
	set_led(LED5, 1);
	playNote(NOTE_C4, 200);
	chThdSleepMilliseconds(300);
	set_rgb_led(LED6, colours[RED].r, colours[RED].g, colours[RED].b);
	chThdSleepMilliseconds(500);

	//Sec 4
	set_led(LED7, 1);
	playNote(NOTE_C4, 200);
	chThdSleepMilliseconds(300);
	set_rgb_led(LED8, colours[RED].r, colours[RED].g, colours[RED].b);
	chThdSleepMilliseconds(500);

	//Sec 5
	clear_leds();
	set_rgb_led(LED2, colours[GREEN].r, colours[GREEN].g, colours[GREEN].b);
	set_rgb_led(LED4, colours[GREEN].r, colours[GREEN].g, colours[GREEN].b);
	set_rgb_led(LED6, colours[GREEN].r, colours[GREEN].g, colours[GREEN].b);
	set_rgb_led(LED8, colours[GREEN].r, colours[GREEN].g, colours[GREEN].b);
	playNote(NOTE_G4, 500);
	chThdSleepMilliseconds(500);

	clear_leds();
}

void indicate_start_follow_walls(){

	set_rgb_led(LED2, colours[BLUE].r, colours[BLUE].g, colours[BLUE].b);
	set_rgb_led(LED4, colours[BLUE].r, colours[BLUE].g, colours[BLUE].b);
	set_rgb_led(LED6, colours[BLUE].r, colours[BLUE].g, colours[BLUE].b);
	set_rgb_led(LED8, colours[BLUE].r, colours[BLUE].g, colours[BLUE].b);
	playNote(NOTE_C4, 100);
	chThdSleepMilliseconds(50);

	playNote(NOTE_E4, 100);
	chThdSleepMilliseconds(50);

	playNote(NOTE_G4, 100);
	chThdSleepMilliseconds(50);

	clear_leds();
	playNote(NOTE_C5, 100);
}

void indicate_end(){
	chThdSleepMilliseconds(1000);
	playMelody(MARIO_FLAG, ML_SIMPLE_PLAY, NULL);
	for(uint16_t i = 0; i < NB_COLOURS; i++){
		set_rgb_led(LED2, colours[i].r, colours[i].g, colours[i].b);
		chThdSleepMilliseconds(200);
		set_rgb_led(LED4, colours[i].r, colours[i].g, colours[i].b);
		chThdSleepMilliseconds(200);
		set_rgb_led(LED6, colours[i].r, colours[i].g, colours[i].b);
		chThdSleepMilliseconds(200);
		set_rgb_led(LED8, colours[i].r, colours[i].g, colours[i].b);
		chThdSleepMilliseconds(200);
	}
	chThdSleepMilliseconds(1000);
}
