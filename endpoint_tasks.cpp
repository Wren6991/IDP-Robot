#include "endpoint_tasks.h"

#include "line_following.h"
#include "robot_state.h"
#include "sensors_actuators.h"
#include "navigation.h"

#include <cstdlib>


void start_box_task(robot_state &state)
{
	if (state.eggs_processed > 0)
	{
		move(state, 0, 0);
		exit(0);
	}
}

void set_next_target(robot_state &state)
{
	if (state.have_egg)
	{
		if (state.have_chick)
			state.target = state.map->vs[VERT_CHICK_BOX];
		else if (state.have_white)
			state.target = state.map->vs[VERT_FRYING_PAN];
		else
			state.target = state.map->vs[VERT_EGG_BOX];
	}
	else
	{
		switch(state.eggs_processed)
		{
		case 0:
			state.target = state.map->vs[VERT_EGG_0];
			break;
		case 1:
			state.target = state.map->vs[VERT_EGG_1];
			break;
		case 2:
			state.target = state.map->vs[VERT_EGG_2];
			break;
		case 3:
			state.target = state.map->vs[VERT_EGG_3];
			break;
		case 4:
			// if time_left < 20 go to start box
			// else
			state.target = state.map->vs[VERT_EGG_4];
			break;
		default:
			// We're finished!
			state.target = state.map->vs[VERT_START];
			break;
		}
	}
}

void egg_task(robot_state &state)
{
	// Align with podium
		// due to length of robot, need to use the "other" end of the line
		// turn west, follow this line until no line detected. Keep going a bit further.
		// turn 180* until line detected again. Follow line back up to egg junction
		// Go through junction, and now use edge following to slowly approach the podium
		// (edge following has less slack than line following)
		// Edge follow until one whisker hits the wall, then drive the opposite motor until both
		// whiskers are on wall

	// Close claw, back away
	// have_egg = true

	// if claw can't close, egg is fake. can skip to realignment step.

	// Split claw (dropping contents into bucket)
	// Use LDR to identify contents.
	// set have_chick or have_white accordingly

	// realign to track
	// (may possibly have to set current vertex to egg_n+1 due to turning during realignment)

	// set_next_target();
}

void egg_box_task(robot_state &state)
{
	// align to box  (90* turn_to_line, drive forward until whiskers detect box)

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
	// have_white = false
	// set_next_target();
}

