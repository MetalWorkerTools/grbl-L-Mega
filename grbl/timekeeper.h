#ifndef timekeeper_h
#define timekeeper_h

#define TIMER_TICS_PER_MINUTE		  15000000UL								// Timer 5, prescaler 1/64, 4 usec / tic.  The number of timer tics between spindle index pulses when running at 1 RPM.

#include "grbl.h"

void timekeeper_init();
uint32_t get_timer_ticks_passed();
void timekeeper_reset();
void debounce_index_pulse();
void debounce_sync_pulse();

uint32_t get_timer_ticks();

#endif