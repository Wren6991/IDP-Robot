#include "endpoint_tasks.h"

#include "line_following.h"
#include "robot_state.h"
#include "sensors_actuators.h"
#include "navigation.h"

#include <cstdlib>
#include <iostream>
#include <robot_delay.h>

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

void wait_for_crossing(robot_state &state)
{
	while (state.line_state != LINE_NONE_DETECTED)
		update_sensor_values(state);
	while (state.line_state < -2 || state.line_state > 2)
		update_sensor_values(state);
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

	if (state.eggs_processed >= 1 && state.eggs_processed <= 3)
	{
		std::cout << "Turning to west\n";
		move(state, -1, -0.5);
		delay(600);
		move(state, 1, -1);
		wait_for_crossing(state);

		move(state, 1, 0);
		delay(300);
		move(state, 0.5, 1);
		std::cout << "Turning east\n";
		wait_for_crossing(state);

		std::cout << "Alignment reached.\n";
	}

	open_claw(state);

	state.integral = 0.f;
	debug_dump(state);

	while (!(state.bump_left || state.bump_right))
	{
		update_sensor_values(state);
		follow_line_ignore_junctions(state);
	}

	std::cout << " Bumped into wall:  " << (state.bump_left ? "l" : "-") << (state.bump_right ? "r" : "-") << "\n";
	move(state, 0, 0);

	// Close claw, back away
	close_claw(state);
	delay(500);
	move(state, -1, 0);
	delay(500);
	move(state, 0, 0);
	state.have_egg = true;
	update_status_leds(state);

	// if claw can't close, egg is fake. can skip to realignment step.
	if (true || state.claw_closed)
	{	
	// Split claw (dropping contents into bucket)
	// Use LDR to identify contents.
	// set have_chick or have_white accordingly
		widen_claw(state);
		delay(1000);
		for (int i = 0; i < 3; ++i)
		{
			narrow_claw(state);
			delay(250);
			widen_claw(state);
			delay(350);
		}
		for (int i = 0; i < 3; ++i)
		{
			move(state, 0, 1);
			delay(300);
			move(state, 0, -1);
			delay(300);
		}
		move(state, 0, 0);
		read_ldr(state);
		if (state.light_sensor > 0.2f)
			state.have_white = true;
		else
			state.have_chick = true;
		update_status_leds(state);
		narrow_claw(state);
	}

	std::cout << "Egg picked up, realigning to track\n";
	// realign to track
	// (may possibly have to set current vertex to egg_n+1 due to turning during realignment)
	move(state, 0, 1);
	wait_for_crossing(state);

	//set_next_target();
}

void egg_box_task(robot_state &state)
{
	// align to box

	move(state, -1, 0);
	delay(1300);
	move(state, 0, -1);
	delay(500);
	move(state, 1, 0);
	delay(500);

	// flap the flapper
	flap_flapper(state);
	unflap_flapper(state);

	// realign to track
	move(state, -1, 1);
	wait_for_crossing(state);
	state.integral = 0.f;

	while (state.line_state != LINE_JUNCTION)
	{
		follow_line_ignore_junctions(state);
		update_sensor_values(state);
	}
	turn_to_line(state, 90.f);

	state.have_white = false;
	update_status_leds(state);
	set_next_target(state);


}

void chick_box_task(robot_state &state)
{
	// align to box

	move(state, -1, 0);
	delay(1300);
	move(state, 0, -1);
	delay(600);
	move(state, 1, 0);
	delay(500);

	// flap the flapper
	flap_flapper(state);
	unflap_flapper(state);

	// realign to track
	move(state, -1, 1);
	wait_for_crossing(state);
	state.integral = 0.f;

	while (state.line_state != LINE_JUNCTION)
	{
		follow_line_ignore_junctions(state);
		update_sensor_values(state);
	}

	state.have_white = false;
	update_status_leds(state);
	set_next_target(state);

}

void frying_pan_task(robot_state &state)
{
	// align to box

	move(state, -1, 0);
	delay(1250);
	move(state, 0, -1);
	delay(500);
	move(state, 1, 0);
	delay(500);

	// flap the flapper
	flap_flapper(state);
	unflap_flapper(state);

	// realign to track
	move(state, -1, 1);
	wait_for_crossing(state);
	state.integral = 0.f;

	while (state.line_state != LINE_JUNCTION)
	{
		follow_line_ignore_junctions(state);
		update_sensor_values(state);
	}

	state.have_white = false;
	update_status_leds(state);
	set_next_target(state);
}

