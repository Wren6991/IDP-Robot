#include "robot_state.h"

#include <cstring>
#include "navigation.h"

robot_state::robot_state()
{
	std::memset(this, 0, sizeof(*this));
	link = new robot_link;
	map = new world_map;
}

robot_state::~robot_state()
{
	delete link;
	delete map;
}
