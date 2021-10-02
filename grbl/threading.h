/*
  threading.h - Handles threading command G33
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

#ifndef threading_h
#define threading_h

extern volatile uint8_t threading_exec_flags;							// Global real time executor bitflag variable for spindle synchronization.
extern volatile uint8_t threading_sync_pulse_count;						// Global synchronization pulse counter
extern volatile uint8_t threading_index_pulse_count;					// Global index pulse counter
extern volatile uint32_t threading_sync_Last_timer_tics;				// Time at last sync pulse
extern volatile uint32_t threading_sync_timer_tics_passed;				// Time passed sync pulse
extern volatile uint32_t threading_index_Last_timer_tics;				// Time at last index pulse
extern volatile uint32_t threading_index_timer_tics_passed;				// Time passed index pulse
extern volatile uint32_t threading_index_spindle_speed;					// The spindle speed calculated from the spindle index pulses. Used for displaying the real spindle speed.
extern volatile float threading_millimeters_target;						// The threading target feed to go as reported by the planner
extern volatile float synchronization_millimeters_error;				// The threading feed error calculated at every synchronization pulse
extern float threading_mm_per_synchronization_pulse;					// Z-axis motion at each sync pulse. Is not declared as volatile because it is not updated by an ISR routine.
extern float threading_mm_per_index_pulse;								// Z-axis motion at each index pulse. Is not declared as volatile because it is not updated by an ISR routine.
extern float threading_feed_rate_calculation_factor;					// Factor is used in plan_compute_profile_nominal_speed() and is calculated at threading start for performance reasons.

void threading_init(float K_value);										// Initializes the G33 threading pass using the K value set in the gcode
uint32_t timer_tics_passed_since_last_index_pulse();
void process_spindle_index_pulse();
void process_spindle_synchronization_pulse();
void update_planner_feed_rate();
void threading_reset();
bool spindle_synchronization_active();
bool index_pulse_active();
bool sync_pulse_active();
#endif