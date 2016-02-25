#include <iostream>
#include <robot_delay.h>
#include <robot_instr.h>
#include <robot_link.h>

#include "robot_state.h"
#include "navigation.h"
#include "sensors_actuators.h"
#include "line_following.h"

#include <cmath>
#include <stdlib.h>

#ifdef __arm__
	#define ROBOT_NUM "127.0.0.1"
#else
	#define ROBOT_NUM 50
#endif

int main(int argc, char **argv)
{
	robot_state state;
	robot_link *link = state.link;

	if (!link->initialise(ROBOT_NUM))
	{
		std::cout << "Cannot initialise link\n";
		link->print_errs("    ");
		return -1;
	}

	std::cout << "Connected\n";

	init_sensors(state);

	move(state, 0, 0);

	state.current = state.map->vs[VERT_START];
	state.target = state.map->vs[VERT_FRYING_PAN];
	state.current_path = state.map->find_path(state.current, state.target);

	vertex *current = state.current;
	for (size_t i = 0; i < state.current_path.size(); ++i)
	{
		if (!current)
			break;
		std::cout << "(" << current->posx << ", " << current->posy << "),\n";
		current = state.current_path[i]->other(current);

		if (current != state.target)
			std::cout << "Turn by " << state.map->turning_angle(state.current_path[i], state.current_path[i + 1], current) << "* at ";
	}
	std::cout << "(" << current->posx << ", " << current->posy << ")\n";

	float lasttime = 0;
	while (true)
	{
		update_sensor_values(state);
		follow_line(state);
		/*float time = state.watch.read();
		std::cout << time - lasttime << "\n";
		lasttime = time;*/
		/*if (rand() / (float)RAND_MAX < 0.05f)
			debug_dump(state);*/
	}

	/*vertex *current, *target;

	while (true)
	{
		update_sensor_values(state);
		follow_line(state);

		// Perform the task associated with the goal we have reached,
		// and update the target:

		if(state.current == state.target)
			state.target->endpoint_task(state);
	}*/
}
