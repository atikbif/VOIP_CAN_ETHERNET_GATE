/*
 * button_led.h
 *
 *  Created on: 17 џэт. 2019 у.
 *      Author: Roman
 */

#ifndef BUTTON_LED_H_
#define BUTTON_LED_H_

#define		OFF		0
#define 	GREEN	1
#define		RED		2

void led_init(void);
void button_init(void);

void set_first_led(unsigned char color);
void toggle_first_led(unsigned char color);
void set_second_led(unsigned char color);
void toggle_second_led(unsigned char color);
unsigned char get_button_state(void);

#endif /* BUTTON_LED_H_ */
