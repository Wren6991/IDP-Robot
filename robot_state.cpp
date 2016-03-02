#include "robot_state.h"

#include <cstring>
#include "navigation.h"

robot_state::robot_state()
{
	std::memset(this, 0, sizeof(*this));
	current_dirx = 1.f;
	link = new robot_link;
	map = new world_map;
	watch.start();
}

robot_state::~robot_state()
{
	delete link;
	delete map;
}
