#include "endpoint_tasks.h"

#include "line_following.h"
#include "robot_state.h"
#include "sensors_actuators.h"

void set_next_target(robot_state &state)
{
	if (state.have_egg)
	{
		if (state.have_chick)
			state.target = VERT_CHICK_BOX;
		else if (state.have_egg_white)
			state.target = VERT_FRYING_PAN;
		else
			state.target = VERT_EGG_BOX;
	}
	else
	{
		switch(state.eggs_processed)
		{
		case 0:
			state.target = VERT_EGG_0;
			break;
		case 1:
			state.target = VERT_EGG_1;
			break;
		case 2:
			state.target = VERT_EGG_2;
			break;
		case 3:
			state.target = VERT_EGG_3;
			break;
		case 4:
			state.target = VERT_EGG_4;
			break;
		default:
			// We're finished!
			state.target = state.current;
			break;
		}
	}
}

void egg_task(robot_state &state)
{
	// Align with podium

	// Close claw, back away
	// have_egg = true

	// if claw can't close, egg is fake. can skip to realignment.

	// Split claw (dropping contents into bucket)
	// Use LDR to identify contents.
	// set have_chick or have_egg_white accordingly

	// realign to track
	// (may possibly have to set current vertex to egg_n+1 due to turning during realignment)

	// set_next_target();
}

void egg_box_task(robot_state &state)
{
	// align to box

	// ensure ramp/flapper is extended
	// unsplit claw
	// open claw
	// have_egg = false

	// realign to track
	// set_next_target();
}

void chick_box_task(robot_state &state)
{
	// align to box

	// flap the flapper

	// realign to track
	// have_chick = false
	// set_next_target();
}

void frying_pan_task(robot_state &state)
{
	// align to box

	// flap the flapper

	// realign to track
	// have_egg_white = false
	// set_next_target();
}

