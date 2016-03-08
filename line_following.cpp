#include "line_following.h"

#include <cmath>
#include <iostream>
#include <robot_delay.h>

#include "robot_state.h"
#include "sensors_actuators.h"
#include "navigation.h"

// Leftmost sensor is MSB of index, rightmost is LSB
const line_state_t sensor_to_line[16] = {
	LINE_NONE_DETECTED,	// 0000
	LINE_LEFT_3,		// 0001
	LINE_LEFT_2,		// 0010
	LINE_LEFT_2,		// 0011
	LINE_RIGHT_2,		// 0100
	LINE_LEFT_1,		// 0101
	LINE_STRAIGHT,		// 0110
	LINE_LEFT_1,		// 0111
	LINE_RIGHT_3,		// 1000
	LINE_JUNCTION,		// 1001
	LINE_RIGHT_1,		// 1010
	LINE_JUNCTION,		// 1011
	LINE_RIGHT_2,		// 1100
	LINE_JUNCTION,		// 1101
	LINE_RIGHT_1,		// 1110
	LINE_JUNCTION		// 1111
};

line_state_t line_state_from_sensors(robot_state &state)
{
	int line_bits = (state.line_sens[0] ? 0x01 : 0x00) |
		(state.line_sens[1] ? 0x02 : 0x00) |
		(state.line_sens[2] ? 0x04 : 0x00) |
		(state.line_sens[3] ? 0x08 : 0x00);
	return sensor_to_line[line_bits];
}

void follow_line(robot_state &state)
{
	state.at_junction = state.line_state == LINE_JUNCTION;

	if (state.line_state == LINE_JUNCTION)
	{
		state.integral = 0.f;
		// Don't do anything if we're already at the target
		if (state.current == state.target)
		{
			move(state, 0, 0);
			return;
		}		
		// Advance our state along the edge to next vertex on route
		// And store the new direction for entering this vertex
		edge *last_edge;
		do
		{
			last_edge = state.current_path[0];
			state.current = last_edge->other(state.current);
			state.current_dirx = last_edge->dirx(state.current);
			state.current_diry = last_edge->diry(state.current);
			std::cout << "Arrived at junction (" << state.current->posx << ", " << state.current->posy << ")\n";
			if (state.current->ignore_junction)
				std::cout << "(Ignoring this junction ^ )\n";
			// Pop the edge we just traversed off the front of the path
			// SHOULD NOT USE VECTOR, better a deque
			state.current_path.erase(state.current_path.begin());
		} while (state.current->ignore_junction);
		// Stop if now at target
		if (state.current == state.target)
		{
			move(state, 0, 0);
			return;
		}
		// Find turning angle to next edge, turn onto it
		turn_to_line(state, state.map->turning_angle(
			last_edge,
			state.current_path[0],
			state.current));

	}
	else if (state.line_state == LINE_NONE_DETECTED)
	{
		state.integral = 0.f;
		// turn away from line
		// reverse until line at centre
		// resume normal following
		/*move(state, -1, 0);
		while (state.line_state == LINE_NONE_DETECTED)
		{
			update_sensor_values(state);
			state.line_state = line_state_from_sensors(state);
		}*/
		//std::cout << "No line detected\n";
		move(state, 0, 0);
	}
	else
	{
		follow_line_ignore_junctions(state);
	}
}

// e.g. lining up to egg bays
void follow_line_ignore_junctions(robot_state &state)
{
	static float lasttime = 0.f;

	float time = state.watch.read();
	float dt = (time - lasttime) / 1000.f; // We want s not ms!
	lasttime = time;
	if (state.line_state >= -3 && state.line_state <= 3)
	{
		// we have a line state from -3 to 3, from completely left to completely right
		state.integral += state.line_state * dt * 0.3f;
		move(state, 1, -1.f * (state.line_state + state.integral)/3.f);	// full speed ahead!
	}
	else if (state.line_state == LINE_JUNCTION)
	{
		move(state, 1, 0);
	}
	else
	{
		move(state, 0, 0);
	}
}

void follow_edge(robot_state &state, int sensor_no, bool flip)
{
	bool s = state.line_sens[sensor_no];
	if ((s && !flip) || (!s && flip))
		move(state, 1, -1);
	else
		move(state, 1, 1);
	set_led(state, LED_EGG, (s && !flip) || (!s && flip));
}

void turn_to_line(robot_state &state, float degrees_approx)
{
	std::cout << "Making " << degrees_approx << "* turn.\n";
	if (fabs(degrees_approx) < 5.f)	// the junction is straight, ascribe to numerical error
	{
		move(state, 1, 0);
		while (state.line_state == LINE_JUNCTION)
			update_sensor_values(state);
		delay(50);	// Just to make sure we're past the junction
	}
	else
	{
		float turn = degrees_approx < 0 ? 1 : -1;
		// Turn in desired direction until we are off the line
		move(state, -0.2, turn);
		while (state.line_state != LINE_NONE_DETECTED)
			update_sensor_values(state);
		std::cout << "Left previous line\n";
		// We've turned off the line -- now just a bit extra:
		//delay(100);
		// Go forward until we hit the new line.
		move(state, 1, 0);
		while (state.line_state == LINE_NONE_DETECTED)
			update_sensor_values(state);

		std::cout << "line acquired, using edge following to align...\n";
		// Now use edge following to acquire line
		while (state.line_state != LINE_STRAIGHT)
		{
			if (turn < 0)
				follow_edge(state, 3, false);
			else
				follow_edge(state, 0, true);
			update_sensor_values(state);
		}
		std::cout << "Realigned to line, resuming normal following\n";
	}
}
			








