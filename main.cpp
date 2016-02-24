#include <iostream>
#include <robot_delay.h>
#include <robot_instr.h>
#include <robot_link.h>

#include "robot_state.h"
#include "navigation.h"
#include "sensors_actuators.h"
#include "line_following.h"

#include <cmath>

#ifdef __arm__
	#define ROBOT_NUM "127.0.0.1"
#else
	#define ROBOT_NUM 50
#endif

int main(int argc, char **argv)
{
	robot_state state;
	robot_link *link = state.link;

	vertex *start = state.map->vs[VERT_EGG_0];
	vertex *end = state.map->vs[VERT_EGG_BOX];
	std::vector<edge*> edges = state.map->find_path(start, end);

	vertex *current = start;
	for (size_t i = 0; i < edges.size(); ++i)
	{
		if (!current)
			break;
		std::cout << "(" << current->posx << ", " << current->posy << "),\n";
		current = edges[i]->other(current);

		if (current != end)
			std::cout << "Turn by " << state.map->turning_angle(edges[i], edges[i + 1], current) << "*\n";
	}
	std::cout << "(" << current->posx << ", " << current->posy << ")\n";

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
	
	update_status_leds(state);
	delay(1000);
	while (true)
	{
		/*state.have_egg = true;
		state.have_white = true;
		state.have_chick = false;
		update_status_leds(state);
		open_claw(state);
		narrow_claw(state);
		set_led(state, LED_LDR, true);
		std::cout << "1\n";
		delay(250);
		state.have_egg = false;
		state.have_white = false;
		state.have_chick = true;
		update_status_leds(state);
		close_claw(state);
		widen_claw(state);
		set_led(state, LED_LDR, false);
		std::cout << "2\n";
		delay(250);*/
		read_ldr(state);
		std::cout << "Light level: " << state.light_sensor << "\n";
	}

	while (true)
	{
		update_sensor_values(state);
		follow_line(state);
		read_ldr(state);
		debug_dump(state);
		std::cout << "light: " << state.light_sensor << "\n";
		delay(100);
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
