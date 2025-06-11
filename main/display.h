#ifndef DISPLAY_H
#define DISPLAY_H
#include <stdbool.h>
#include <stdint.h>

extern uint8_t digit_color[4][3];
extern uint8_t bar_color[3];
extern uint8_t digit_brightness;
extern uint8_t bar_brightness;
extern uint8_t torch_brightness;
extern bool torch_mode;
extern bool blink_last_dot;

void display_init(void);

#endif // DISPLAY_H
