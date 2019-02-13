/*
 * frame_stack.c
 *
 *  Created on: 12 февр. 2019 г.
 *      Author: Roman
 */

#include "frame_stack.h"
#include "button_led.h"

frame frames[FRAME_COUNT];

static unsigned char wr_pos = 0;

static unsigned char empty_buf[20] = {0x1E,0x9D,0x66,0x00,0x00,0x39,0xCE,0x70,0x00,0x1C,0xE7,0x38,0x00,0x0E,0x73,0x9C,0x00,0x07,0x39,0xCE};

void init_frames() {
	static unsigned char i = 0;
	static unsigned char j = 0;
	for(i=0;i<FRAME_COUNT;i++) {
		frames[i].ready = 0;
		for(j=0;j<20;j++) {
			frames[i].buf[j] = empty_buf[j];
		}
	}
}


void add_frame(unsigned char *ptr) {
	static unsigned char i = 0;
	static unsigned char j = 0;

	// сброс при заполнении буфера
	if(wr_pos>=FRAME_COUNT) {
		toggle_second_led(RED);
		wr_pos = 0;
		for(i=0;i<FRAME_COUNT;i++) {
			frames[i].ready = 0;
			for(j=0;j<20;j++) {
				frames[i].buf[j] = empty_buf[j];
			}
		}
	}
	for(i=0;i<20;i++) {
		frames[wr_pos].buf[i] = ptr[i];
	}
	frames[wr_pos].ready=1;
	wr_pos++;
}

unsigned char get_frame(unsigned char *ptr) {
	static unsigned char i = 0;
	static unsigned char j = 0;

	if(wr_pos<3) return 0;
	if((wr_pos<FRAME_COUNT) && frames[0].ready) {
		for(i=0;i<20;i++) ptr[i] = frames[0].buf[i];
		for(i=0;i<FRAME_COUNT-1;i++) {
			for(j=0;j<20;j++) {
				frames[i].buf[j] = frames[i+1].buf[j];
				frames[i].ready = frames[i+1].ready;
			}
		}
		for(i=0;i<20;i++) frames[FRAME_COUNT-1].buf[i] = empty_buf[i];
		if(wr_pos) wr_pos--;
		return 1;
	}
	return 0;
}
