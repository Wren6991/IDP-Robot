#include <iostream>
#include <robot_delay.h>
#include <robot_instr.h>
#include <robot_link.h>

#include "robot_state.h"
#include "navigation.h"
#include "sensors_actuators.h"
#include "line_following.h"

#include "endpoint_tasks.h" // should not be here

#include <cmath>
#include <stdlib.h>

#ifdef __arm__
	#define ROBOT_NUM "127.0.0.1"
#else
	#define ROBOT_NUM 50
#endif

void mech_demo(robot_state &state)
{
	close_claw(state);
	delay(1000);
	widen_claw(state);
	delay(1000);

	narrow_claw(state);
	delay(250);
	widen_claw(state);
	delay(250);
	narrow_claw(state);
	delay(250);
	widen_claw(state);
	delay(250);
	narrow_claw(state);
	delay(250);
	widen_claw(state);
	delay(1000);


	state.link->command(MOTOR_PADDLE_GO, 127);
	delay(1000);
	state.link->command(MOTOR_PADDLE_GO, 255);
	delay(1000);
	state.link->command(MOTOR_PADDLE_GO, 0);		
	open_claw(state);
	delay(500);
	narrow_claw(state);
	delay(3000);
}

int main(int argc, char **argv)
{
	robot_state state;
	robot_link *link = state.link;

	state.current = state.map->vs[VERT_START];
	state.target = state.map->vs[VERT_EGG_0];
	state.current_path = state.map->find_path(state.current, state.target);

	vertex *current = state.current;
	for (size_t i = 0; i < state.current_path.size(); ++i)
	{
		if (!current)
			break;
		std::cout << "(" << current->posx << ", " << current->posy << "),\n";
		current = state.current_path[i]->other(current);
	}
	std::cout << "(" << current->posx << ", " << current->posy << ")\n";

	if (!link->initialise(ROBOT_NUM))
	{
		std::cout << "Cannot initialise link\n";
		link->print_errs("    ");
		return -1;
	}

	std::cout << "Connected\n";

	init_sensors_actuators(state);

	//move(state, 0, 0);

	while (true)
	{
		update_sensor_values(state);
		follow_line(state);
		if (state.current == state.target)
		{
			state.eggs_processed = 0;
			egg_task(state);
			break;
		}
	}
}



