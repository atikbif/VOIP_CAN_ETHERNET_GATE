/*
 * frame_stack.c
 *
 *  Created on: 12 февр. 2019 г.
 *      Author: Roman
 */

#include "frame_stack.h"
#include "button_led.h"

frame frames[FRAME_COUNT];
frame can_frames[FRAME_COUNT];

static unsigned char wr_pos = 0;
static unsigned char can_wr_pos = 0;

unsigned char empty_buf[7] = {0x08,0x0B,0xE4,0xB9,0x9C,0x34,0x04};

void init_frames() {
	static unsigned char i = 0;
	static unsigned char j = 0;
	for(i=0;i<FRAME_COUNT;i++) {
		frames[i].ready = 0;
		can_frames[i].ready = 0;
		for(j=0;j<7;j++) {
			frames[i].buf[j] = empty_buf[j];
			frames[i].length = 7;
			can_frames[i].buf[j] = empty_buf[j];
			can_frames[i].length = 7;
		}
	}
}


void add_frame(unsigned char *ptr, unsigned char length) {
	static unsigned char i = 0;
	static unsigned char j = 0;
	if(length==0) return;

	// сброс при заполнении буфера
	if(wr_pos>=FRAME_COUNT) {
		toggle_second_led(RED);
		wr_pos = 0;
		for(i=0;i<FRAME_COUNT;i++) {
			frames[i].ready = 0;
			frames[i].length = 0;
			for(j=0;j<7;j++) {
				frames[i].buf[j] = empty_buf[j];
			}
			frames[i].length = 7;
		}
	}
	for(i=0;i<length;i++) {
		frames[wr_pos].buf[i] = ptr[i];
	}
	frames[wr_pos].length = length;
	frames[wr_pos].ready=1;
	wr_pos++;
}

unsigned char get_frame(unsigned char *ptr) {
	static unsigned char i = 0;
	static unsigned char j = 0;
	unsigned char length = 0;

	if(wr_pos<3) return 0;
	if((wr_pos<FRAME_COUNT) && frames[0].ready) {
		for(i=0;i<frames[0].length;i++) ptr[i] = frames[0].buf[i];
		length = frames[0].length;
		for(i=0;i<FRAME_COUNT-1;i++) {
			frames[i].ready = frames[i+1].ready;
			frames[i].length = frames[i+1].length;
			for(j=0;j<40;j++) {
				frames[i].buf[j] = frames[i+1].buf[j];
			}
		}
		for(i=0;i<7;i++) frames[FRAME_COUNT-1].buf[i] = empty_buf[i];
		frames[FRAME_COUNT-1].length = 7;
		if(wr_pos) wr_pos--;
	}
	return length;
}

void add_can_frame(unsigned char *ptr, unsigned char length) {
	static unsigned char i = 0;
	static unsigned char j = 0;

	// сброс при заполнении буфера
	if(can_wr_pos>=FRAME_COUNT) {
		toggle_second_led(RED);
		can_wr_pos = 0;
		for(i=0;i<FRAME_COUNT;i++) {
			can_frames[i].ready = 0;
			for(j=0;j<7;j++) {
				can_frames[i].buf[j] = empty_buf[j];
			}
			can_frames[i].length = 7;
		}
	}
	for(i=0;i<length;i++) {
		can_frames[can_wr_pos].buf[i] = ptr[i];
	}
	can_frames[can_wr_pos].length = length;
	//for(i=0;i<7;i++) frames[FRAME_COUNT-1].buf[i] = empty_buf[i];
	//frames[FRAME_COUNT-1].length = 7;

	can_frames[can_wr_pos].ready=1;
	can_wr_pos++;
}

unsigned char get_can_frame(unsigned char *ptr) {
	static unsigned char i = 0;
	static unsigned char j = 0;
	unsigned char length = 0;

	//toggle_second_led(GREEN);

	if(can_wr_pos<3) return 0;
	if((can_wr_pos<FRAME_COUNT) && can_frames[0].ready) {
		for(i=0;i<can_frames[0].length;i++) ptr[i] = can_frames[0].buf[i];
		length = can_frames[0].length;
		for(i=0;i<FRAME_COUNT-1;i++) {
			can_frames[i].ready = can_frames[i+1].ready;
			can_frames[i].length = can_frames[i+1].length;
			for(j=0;j<40;j++) {
				can_frames[i].buf[j] = can_frames[i+1].buf[j];
			}
		}
		for(i=0;i<7;i++) can_frames[FRAME_COUNT-1].buf[i] = empty_buf[i];
		can_frames[FRAME_COUNT-1].length = 7;
		if(can_wr_pos) can_wr_pos--;
	}
	return length;
}

