/*
  threading.c - Handles threading command G33
  Part of Grbl

  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

volatile uint8_t threading_exec_flags;							// Global realtime executor bitflag variable for spindle synchronization.
volatile uint8_t threading_index_pulse_count;					// Global index pulse counter. Only a few pulses are needed to start looking for the first synchronization pulse, so defined as uint8
volatile uint8_t threading_sync_pulse_count;					// Global synchronization pulse counter. Only a few pulses are needed to start G33 executing, so defined as uint8
volatile uint32_t threading_sync_Last_timer_tics;				// Time at the last sync pulse
volatile uint32_t threading_sync_timer_tics_passed;				// Time passed at the last sync pulse
volatile uint32_t threading_index_Last_timer_tics;				// Time at the last index pulse
volatile uint32_t threading_index_timer_tics_passed=0;	    	// Time passed at the last index pulse
volatile uint32_t threading_index_spindle_speed;				// The measured spindle speed used for G33 initial speed and reporting the real spindle speed
float threading_mm_per_synchronization_pulse;					// The factor to calculate the feed rate from the spindle speed
float threading_mm_per_index_pulse;								// The factor to calculate the feed rate from the spindle speed
volatile float threading_millimeters_target;					// The threading feed target as reported by the planner
volatile float synchronization_millimeters_error;				// The synchronization feed error calculated at every synchronization pulse. It can be reported to check the threading accuracy
float threading_feed_rate_calculation_factor;					// factor is used in plan_compute_profile_nominal_speed(), depends on the number of synchronization pulses and is calculated on startup for performance reasons.

// Initializes the G33 threading pass by resetting the timer, spindle counter,
// setting the current z-position as reference and calculating the (next) target position.
void threading_init(float K_value)
{
	threading_mm_per_synchronization_pulse= K_value / (float) settings.sync_pulses_per_revolution;						// Calculate the global mm feed per synchronization pulse value.
	threading_mm_per_index_pulse= K_value;																				// Calculate the global mm feed per synchronization pulse value.
	threading_feed_rate_calculation_factor  = ((float) 15000000 / (float) settings.sync_pulses_per_revolution);			// Calculate the factor to speedup the planner during threading
	timekeeper_reset();																									// Reset the timekeeper to avoid calculation errors when timer overflow occurs (to be sure)
	threading_reset();																									// Sets the target position to zero and calculates the next target position																					
}
// Reset variables to start the threading
void threading_reset()
{
	threading_index_pulse_count=0;	//set the spindle index pulse count to 0
	threading_index_Last_timer_tics=0;
	threading_sync_Last_timer_tics=0;
	threading_sync_pulse_count=0;
	threading_millimeters_target=0;																			//Set this value to 0, will be update at the start of the planner block
	system_clear_threading_exec_flag(0xff);																	//Clear all the bits to avoid executing
}

//Returns the time in 4 useconds tics since the last index pulse
//To avoid updates while reading, it is handle atomic
uint32_t timer_tics_passed_since_last_index_pulse()
{
	uint32_t tics;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){tics= get_timer_ticks()-threading_index_Last_timer_tics;}				// Use atomic to avoid errors due to timer updates
	return tics;
}
// This routine processes the spindle index pin hit by increasing the index pulse counter and calculating the time between pulses
// This calculated spindle speed is used for showing the actual spindle speed in the report
void process_spindle_index_pulse()
{
	threading_index_timer_tics_passed=get_timer_ticks()-threading_index_Last_timer_tics;		// Calculate the time between index pulses
	threading_index_Last_timer_tics+=threading_index_timer_tics_passed;							// adjust for calculating the next time
	threading_index_pulse_count++;																// Increase the pulse count
	threading_index_spindle_speed = TIMER_TICS_PER_MINUTE / threading_index_timer_tics_passed;	// calculate the spindle speed  at this place (not in the report) reduces the CPU time because a GUI will update more frequently
}

// Processes the synchronization pulses by increasing the synchronization counter and calculating the time between the synchronization pulses
void process_spindle_synchronization_pulse()
{
	threading_sync_timer_tics_passed=get_timer_ticks()-threading_sync_Last_timer_tics;		// Calculate the time between synchronization pulses
	threading_sync_Last_timer_tics+=threading_sync_timer_tics_passed;						// adjust for calculating the next time
	threading_sync_pulse_count++;															// Increase the synchronization pulse count
}

// This routine does the processing needed to keep the Z-axis in sync with the spindle during a threading pass G33
// This is done only, if the current planner block is a G33 motion indicated by the planner condition PL_COND_FLAG_FEED_PER_REV
// Recalculates the feed rates for all the blocks in the planner as if a new block was added to the planner que
// If the current block isn't a G33 motion, the synchronization_millimeters_error will be set to zero
void update_planner_feed_rate() {
  plan_block_t *plan = plan_get_current_block();
  if ((plan!=NULL) && (plan->condition & PL_COND_FLAG_FEED_PER_REV)) {	// update only during threading
    plan_update_velocity_profile_parameters();							// call plan_compute_profile_nominal_speed() that wil calculate the requested feed rate
	plan_cycle_reinitialize();											// update the feed rates in the blocks
  }
  else synchronization_millimeters_error=0;								// set the error to zero so it can be used in reports
}

// returns true if Spindle sync is active otherwise false
bool spindle_synchronization_active()
{
	plan_block_t *plan = plan_get_current_block();
	if ((plan!=NULL) && (plan->condition & PL_COND_FLAG_FEED_PER_REV)) return true;
	return false;
}

// Returns true if the index pulse is active
bool index_pulse_active()
{
#ifndef DEFAULTS_RAMPS_BOARD //On Mega board SYNC pulses are on the INT1 interrupt pin (D3) 
	return limits_get_state(LIMIT_PIN_MASK_Y_AXIS);	// This is the lathe version, Y-axis limit pin hits are spindle index pulses so handle them and do not reset controller
#else
	return bit_isfalse(SPINDLE_INDEX_PIN, SPINDLE_INDEX_MASK);								// spindle index pulses are on INT0 = SCL input, active low
#endif
}

// Returns true if the sync pulse is active
bool sync_pulse_active()
{
	if (settings.sync_pulses_per_revolution==1) return index_pulse_active();				// the index pulse is used as sync pulse
#ifndef DEFAULTS_RAMPS_BOARD //On Mega board SYNC pulses are on the INT1 interrupt pin (D3)
	uint8_t pin = system_control_get_state();
	return bit_istrue(pin,CONTROL_PIN_INDEX_SPINDLE_SYNC);									// spindle sync pulses are on ctrl input
#else
	return bit_isfalse(SPINDLE_SYNC_PIN, SPINDLE_SYNC_MASK);								// spindle sync pulses are on INT1 = SDA input, active low
#endif
}
