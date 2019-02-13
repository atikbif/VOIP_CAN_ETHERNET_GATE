/*
 * frame_stack.h
 *
 *  Created on: 12 февр. 2019 г.
 *      Author: Roman
 */

#ifndef FRAME_STACK_H_
#define FRAME_STACK_H_

#define FRAME_COUNT			10

typedef struct {
	unsigned char buf[20];
	unsigned char ready;
} frame;

void init_frames();
void add_frame(unsigned char *ptr);
unsigned char get_frame(unsigned char *ptr);

#endif /* FRAME_STACK_H_ */
