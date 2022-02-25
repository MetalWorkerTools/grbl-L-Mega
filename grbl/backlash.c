#include <string.h>
//#include "backlash.h"
#include "grbl.h"
/*
* backlash.c
*
*  Author: Jeff Dill
*/

s_back_lash back_lash_compensation;
void backlash_initialize()
{
	//memset(back_lash_compensation.new_comp_direction,0,sizeof(back_lash_compensation.new_comp_direction));
	memset(back_lash_compensation.last_comp_direction,0,sizeof(back_lash_compensation.last_comp_direction));
	memset(back_lash_compensation.comp_per_axis_mm,0,sizeof(back_lash_compensation.comp_per_axis_mm));
}

void backlash_comp(float *target, plan_line_data_t *pl_data)
{
	
	
	int32_t target_steps[N_AXIS];
	int32_t *position_steps;
	
	if (pl_data->condition & PL_COND_FLAG_SYSTEM_MOTION)
	{
		#ifdef COREXY
		position_steps[X_AXIS] = system_convert_corexy_to_x_axis_steps(sys_position);
		position_steps[Y_AXIS] = system_convert_corexy_to_y_axis_steps(sys_position);
		position_steps[Z_AXIS] = sys_position[Z_AXIS];
		#else
		memcpy(position_steps, sys_position, sizeof(sys_position));
		#endif
	}
	else
	{
		position_steps = plan_get_position();
	}
	
	uint8_t idx=0;
	uint8_t needs_comp = 0;
	
	//printString("checking comp loop\r");
	for (idx = 0;idx<N_AXIS;idx++)
	{
		//Must clear this or the next time we do a comp move we will still have any prior axis comps in there!
		back_lash_compensation.comp_per_axis_mm[idx] = 0;// target[idx];
		target_steps[idx] = lround(target[idx] * settings.steps_per_mm[idx]);
		//see if we are moving at all
		int32_t step_diff=(target_steps[idx] - position_steps[idx]);
		//if moving is it a + or - movement
		step_diff = step_diff>0?1:(step_diff<0?-1:0);
		//if last direction is different than the new direction, and its not 0, we need to comp.
		if (back_lash_compensation.last_comp_direction[idx]!=0 && step_diff!=0
		&& (back_lash_compensation.last_comp_direction[idx] !=step_diff))
		{
			//printString("axis ");
			//printInteger(idx);
			//printString(" set for comp");
			back_lash_compensation.comp_per_axis_mm[idx] = settings.backlash_per_axis[idx]*step_diff;

			needs_comp = 1;
		}
	}
	
	if (needs_comp)
	{
		float new_target[N_AXIS];
		memcpy(new_target,back_lash_compensation.comp_per_axis_mm,sizeof(back_lash_compensation.comp_per_axis_mm));
		plan_line_data_t new_plan;
		plan_line_data_t *new_pl = &new_plan;
		memcpy(new_pl,pl_data,sizeof(plan_line_data_t));
		new_plan.condition=(1<<PL_COND_FLAG_BACKLASH_COMP);
		new_plan.condition |= PL_COND_FLAG_RAPID_MOTION;
		print_uint8_base10(new_target[N_AXIS]);
		mc_line(new_target,&new_plan);

	}
	
	
}