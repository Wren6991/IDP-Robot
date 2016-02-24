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
	
	/*for (int speed = -1; speed <= 1; ++speed)
	{
		for (int turn = -1; turn <= 1; ++turn)
		{
			move(state, speed, turn);
			delay(2000);
			move(state, 0, 0);
			delay(500);
		}
	}*/

	move(state, 0, 0);


	while (true)
	{
		update_sensor_values(state);
		follow_line(state);
		if (rand() / (float)RAND_MAX < 0.1f)
			debug_dump(state);
		delay(10);
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
