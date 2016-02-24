#include "line_following.h"
#include "robot_state.h"
#include "sensors_actuators.h"

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
	line_state_t line_state = line_state_from_sensors(state);
	state.line_state = line_state;

	if (line_state == LINE_JUNCTION)
	{
		/*if (state.current == state.target)
			return;
		edge *e = state.current_path[0];
		state.current = e->other(state.current);
		// perform turn
		// update current node in state*/
		move(state, 0, 0);
	}	
	else if (line_state == LINE_NONE_DETECTED)
	{
		// turn away from line
		// reverse until line at centre
		// resume normal following
		move(state, 0, 0);
	}
	else
	{
		// we have a line state from -3 to 3, from completely left to completely right
		// Either bang-bang, proportional or PID control should be okay
		// at least PI would be good for the curved sections
		// e.g. as follows
		move(state, 1, -line_state/3.0001f);	// full speed ahead!
	}
}

void follow_edge(robot_state &state, int sensor_no)
{
	if (state.line_sens[sensor_no])
		move(state, 0.5, -0.5);
	else
		move(state, 0.5, 0.5);
}

void turn_to_line(robot_state &state, float degrees_approx)
{
	if (fabs(degrees_approx) < 5.f)	// the junction is straight, ascribe to numerical error
	{
		move(state, 1, 0);
		while (state.line_state == LINE_JUNCTION)
		{
			read_sensors(state);
			state.line_state = line_state_from_sensors(state);
		}
	}
	else
	{
		turn = degrees_approx < 0 ? -1 : 1;
		// Turn in desired direction until we are off the line
		move(state, 0, turn);
		while (state.line_state != LINE_NO_LINE)
		{
			read_sensors(state);
			state.line_state = line_state_from_sensors(state);
		}
		// We've turned off the line -- now just a bit extra:
		delay(50);
		// Go forward until we hit the new line. The line following routine will realign us.
		move(state, 1, 0);
		while (state.line_state == LINE_NO_LINE)
		{
			read_sensors(state);
			state.line_state = line_state_from_sensors(state);
		}
	}
}
			








