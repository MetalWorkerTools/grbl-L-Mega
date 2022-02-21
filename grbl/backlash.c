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
		memcpy(position_steps, sys_position, sizeof(sys_position));
	}
	else
	{
		position_steps = plan_get_position();
	}
	
	uint8_t idx=0;
	uint8_t needs_comp = 0;
	
	for (idx = 0;idx<N_AXIS;idx++)
	{
		back_lash_compensation.comp_per_axis_mm[idx] = target[idx];
		target_steps[idx] = lround(target[idx] * settings.steps_per_mm[idx]);
		int32_t step_diff=(target_steps[idx] - position_steps[idx]);
		step_diff = step_diff>0?1:(step_diff<0?-1:0);
		if (back_lash_compensation.last_comp_direction[idx]!=0 && step_diff!=0
			&& (back_lash_compensation.last_comp_direction[idx] !=step_diff))
		{
			back_lash_compensation.comp_per_axis_mm[idx] = settings.backlash_per_axis[idx]*step_diff;
			needs_comp = 1;
		}
		else{
			back_lash_compensation.comp_per_axis_mm[idx] = 0;
		}
	}
	if (needs_comp)
	{
		
		float new_target[N_AXIS];
		memcpy(new_target,back_lash_compensation.comp_per_axis_mm,sizeof(back_lash_compensation.comp_per_axis_mm));
		plan_line_data_t new_plan;
		plan_line_data_t *new_pl = &new_plan;
		memcpy(new_pl,pl_data,sizeof(plan_line_data_t));
		new_plan.condition =(1<<PL_COND_FLAG_BACKLASH_COMP);
		new_plan.condition |= PL_COND_FLAG_RAPID_MOTION;
		mc_line(new_target,&new_plan);
	}
}