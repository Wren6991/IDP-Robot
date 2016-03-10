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
	state.current_path = state.map->find_path(state.current, state.target);
}

void wait_for_crossing(robot_state &state, int tolerance = 2)
{
	while (state.line_state != LINE_NONE_DETECTED) 
		update_sensor_values(state);
	while (state.line_state < -tolerance || state.line_state > tolerance)
		update_sensor_values(state);
}

void advance_current_egg_position(robot_state &state)
{
	// Due to realignment we will now be one egg further up than the one that was collected:
	switch(state.eggs_processed)
	{
	case 0:
		state.current = state.map->vs[VERT_EGG_1];
		break;
	case 1:
		state.current = state.map->vs[VERT_EGG_2];
		break;
	case 2:
		state.current = state.map->vs[VERT_EGG_3];
		break;
	case 3:
		state.current = state.map->vs[VERT_EGG_4];
		break;
	case 4:
		state.current = state.map->vs[VERT_EGG_4];
		break;
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

	std::cout << "entered egg task (" << state.eggs_processed << " eggs processed so far)\n";

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
		move(state, 0, -1);
		delay(100);

		std::cout << "Alignment reached.\n";
	}

	open_claw(state);

	state.integral = 0.f;
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
	while (state.line_state != LINE_JUNCTION)
		update_sensor_values(state);
	delay(600);
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
			delay(200);
			widen_claw(state);
			delay(300);
		}
		move(state, 0, 0);
		read_ldr(state);
		if (state.light_sensor > 0.2f)
			state.have_white = true;
		else
			state.have_chick = true;
		std::cout << (state.have_white ? "Egg white " : "Chick ") << "detected.\n";
		update_status_leds(state);

	}

	std::cout << "Egg picked up, realigning to track\n";
	// realign to track
	// (may possibly have to set current vertex to egg_n+1 due to turning during realignment)
	move(state, 1, 1);
	wait_for_crossing(state);
	delay(100);
	std::cout << "On line, straightening...\n";
	move(state, 1, 0);
	while (state.line_state != LINE_STRAIGHT)
	{
		update_sensor_values(state);
	}

	/*move(state, 1, 0.5);
	delay(500);
	
	do
	{
		update_sensor_values(state);
	} while (state.line_state < -1 || state.line_state > 1);
		
	move(state, 0, 0);
	state.integral = 0.f;*/

	/*float time = state.watch.read();

	while (state.watch.read() < time + 3000.f)
	{
		update_sensor_values(state);
		follow_edge(state, 3, false);
	}*/

	narrow_claw(state);
	advance_current_egg_position(state);
	state.map->vs[VERT_ABOVE_EGGS]->ignore_junction = true;
	set_next_target(state);
	std::cout << "Egg task complete\n";
}

// Subroutine to reacquire line after leaving it to deposit in a box
void realign_from_box(robot_state &state)
{
	move(state, -1, 0.8);
	wait_for_crossing(state);

	while (state.line_state != LINE_STRAIGHT)
		update_sensor_values(state);
	state.integral = 0.f;

	while (state.line_state != LINE_JUNCTION)
	{
		follow_line_ignore_junctions(state);
		update_sensor_values(state);
	}
	turn_to_line(state, 0.f);
}

void egg_box_task(robot_state &state)
{
	std::cout << "entered egg box task\n";
	// align to box

	move(state, -1, 0);
	delay(1300);
	move(state, 0, -1);
	delay(500);
	move(state, 1, 0);
	delay(500);

	// eject the shells
	open_claw(state);
	flap_flapper(state);
	unflap_flapper(state);
	close_claw(state);

	state.have_egg = false;
	update_status_leds(state);

	// realign to track
	
	move(state, -1, 0);
	delay(500);
	move(state, 0, 1);
	wait_for_crossing(state);
	move(state, 1, 0);
	delay(1000);
	move(state, 0, 1);
	wait_for_crossing(state);


	state.eggs_processed++;
	
	state.map->vs[VERT_ABOVE_EGGS]->ignore_junction = false;
	set_next_target(state);
	std::cout << "egg box task complete\n";
}

void chick_box_task(robot_state &state)
{
	std::cout << "entered chick box task\n";
	// align to box

	move(state, -1, 0);
	delay(1300);
	move(state, 0, -1);
	delay(650);
	move(state, 1, 0);
	delay(500);

	// flap the flapper
	flap_flapper(state);
	unflap_flapper(state);

	// realign to track
	realign_from_box(state);

	state.have_chick = false;
	update_status_leds(state);

	set_next_target(state);
	std::cout << "chick box task complete\n";

}

void frying_pan_task(robot_state &state)
{
	std::cout << "entered frying pan task\n";
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
	realign_from_box(state);

	state.have_white = false;
	update_status_leds(state);

	set_next_target(state);
	std::cout << "frying pan task complete\n";
}

