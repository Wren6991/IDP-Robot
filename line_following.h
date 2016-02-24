#ifndef _LINE_FOLLOWING_H_
#define _LINE_FOLLOWING_H_

struct robot_state;

typedef enum {
	LINE_LEFT_3 = -3,
	LINE_LEFT_2 = -2,
	LINE_LEFT_1 = -1,
	LINE_STRAIGHT = 0,
	LINE_RIGHT_1 = 1,
	LINE_RIGHT_2 = 2,
	LINE_RIGHT_3 = 3,
	LINE_NONE_DETECTED,
	LINE_JUNCTION
} line_state_t;

line_state_t line_state_from_sensors(robot_state &state);
void follow_line(robot_state &state);
// Dead reckoning plus using the new line for the final alignment:
void turn(robot_state &state, float degrees);

#endif // _LINE_FOLLOWING_H_
