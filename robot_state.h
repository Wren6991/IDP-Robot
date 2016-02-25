#ifndef _ROBOT_STATE_H_
#define _ROBOT_STATE_H_

#include <robot_link.h>
#include <stopwatch.h>
#include <queue>
#include "line_following.h"

struct vertex;
struct edge;
struct world_map;

struct robot_state {
	robot_link *link;

	// Sensor values are updated by sensor routine at max rate.
	static const int N_LINE_SENSORS = 4;
	bool line_sens[N_LINE_SENSORS];
	bool bump_left, bump_right;
	bool claw_closed;
	float light_sensor;

	// State variables are updated at path endpoints (pickup and drop off)
	// If a fake egg is detected at pickup, have_egg will be true and the others false
	bool have_egg;
	bool have_chick;
	bool have_white;
	int eggs_processed;
	stopwatch watch;

	bool at_junction;
	float current_dirx, current_diry;
	vertex *current, *target;
	std::vector<edge*> current_path;
	line_state_t line_state;
	world_map *map;


	robot_state();
	~robot_state();
};

#endif // _ROBOT_STATE_H_
