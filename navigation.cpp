#include "navigation.h"
#include "endpoint_tasks.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>

const float root2 = 1.414;

edge::edge(vertex *a_, vertex *b_, float length_, float dir_ax_, float dir_ay_, float dir_bx_, float dir_by_)
{
	if (a_ == NULL || b_ == NULL)
		return;	// should throw
	a = a_;
	b = b_;
	a->edges.push_back(this);
	b->edges.push_back(this);
	// calculate length if no length specified:
	// (must be specified if path not straight)
	if (length_ < 0)
		length = sqrtf(powf(b->posx - a->posx, 2) + powf(b->posy - a->posy, 2));
	else
		length = length_;
	// default direction: assume edge is straight line, use vector a->b
	// otherwise use supplied values
	if (dir_ax_ == 0 && dir_ay_ == 0)
	{
		dir_ax = a->posx - b->posx;
		dir_ay = a->posy - b->posy;
	}
	else
	{
		dir_ax = dir_ax_;
		dir_ay = dir_ay_;
	}

	if (dir_bx_ == 0 && dir_by_ == 0)
	{
		dir_bx = b->posx - a->posx;
		dir_by = b->posy - a->posy;
	}
	else
	{
		dir_bx = dir_bx_;
		dir_by = dir_by_;
	}
}

world_map::world_map()
{
	vs.push_back(new vertex(600 * (1 - root2/2), 1200 + 600 * root2/2));
	vs.push_back(new vertex(900, 1800));
	vs.push_back(new vertex(1200 + 600 * root2/2, 1200 + 600 * root2/2));
	vs.push_back(new vertex(0, 1200));
	vs.push_back(new vertex(600, 1200, true));
	vs.push_back(new vertex(900, 1200));
	vs.push_back(new vertex(1200, 1200));
	vs.push_back(new vertex(1800, 1200, true));
	vs.push_back(new vertex(900, 800));
	vs.push_back(new vertex(1800, 800));
	vs.push_back(new vertex(1800, 600));
	vs.push_back(new vertex(1800, 400));
	vs.push_back(new vertex(1800, 200));
	vs.push_back(new vertex(0, 200));
	vs.push_back(new vertex(0, 0));
	vs.push_back(new vertex(200, 0));
	vs.push_back(new vertex(900, 0));
	vs.push_back(new vertex(1800, 0));

	es.push_back(new edge(vs[0], vs[1], M_PI / 4.f * 600 + 300, -1, -1, 1, 0));
	es.push_back(new edge(vs[0], vs[3], M_PI / 4.f * 600, 1, 1, 0, -1));
	es.push_back(new edge(vs[0], vs[4]));
	es.push_back(new edge(vs[1], vs[2], M_PI / 4.f * 600 + 300, -1, 0, 1, -1));
	es.push_back(new edge(vs[1], vs[5]));
	es.push_back(new edge(vs[2], vs[6]));
	es.push_back(new edge(vs[2], vs[7], M_PI / 4.f * 600, -1, 1, 0, -1));
	es.push_back(new edge(vs[3], vs[4]));
	es.push_back(new edge(vs[3], vs[13]));
	es.push_back(new edge(vs[4], vs[5]));
	es.push_back(new edge(vs[5], vs[6]));
	es.push_back(new edge(vs[5], vs[8]));
	es.push_back(new edge(vs[6], vs[7]));
	es.push_back(new edge(vs[7], vs[9]));
	//es.push_back(new edge(vs[8], vs[9]));
	es.push_back(new edge(vs[8], vs[16]));
	es.push_back(new edge(vs[9], vs[10]));
	es.push_back(new edge(vs[10], vs[11]));
	es.push_back(new edge(vs[11], vs[12]));
	es.push_back(new edge(vs[12], vs[17]));
	es.push_back(new edge(vs[13], vs[14]));
	es.push_back(new edge(vs[14], vs[15]));
	es.push_back(new edge(vs[15], vs[16]));
	es.push_back(new edge(vs[16], vs[17]));

	vs[VERT_EGG_0]->endpoint_task = egg_task;
	vs[VERT_EGG_1]->endpoint_task = egg_task;
	vs[VERT_EGG_2]->endpoint_task = egg_task;
	vs[VERT_EGG_3]->endpoint_task = egg_task;
	vs[VERT_EGG_4]->endpoint_task = egg_task;
	vs[VERT_FRYING_PAN]->endpoint_task = frying_pan_task;
	vs[VERT_CHICK_BOX]->endpoint_task = chick_box_task;
	vs[VERT_EGG_BOX]->endpoint_task = egg_box_task;
	vs[VERT_START]->endpoint_task = start_box_task;
}

float heuristic_distance(vertex *a, vertex *b)
{
	return fabs(a->posx - b->posx) + fabs(a->posy - b->posy);
}

struct f_lessthan
{
	bool operator()(vertex * const& a, vertex * const& b)
	{
		return a->f_score < b->f_score;
	}
};

std::vector<edge*> world_map::reconstruct_path(vertex *start, vertex *end)
{
	std::vector<edge*> path_edges;

	vertex *current = end;

	// backtrack from end to start using v->came_from
	while(current && current != start)
	{
		path_edges.push_back(current->came_from);
		current = current->came_from->other(current);
	}
	std::reverse(path_edges.begin(), path_edges.end());

	return path_edges;
}

std::vector<edge*> world_map::find_path(vertex *start, vertex *end)
{
	std::cout << "Beginning pathfinding\n";
	std::set<vertex*> closed_set;	// fully determined
	std::set<vertex*> open_set;		// in consideration

	open_set.insert(start);

	for (size_t i = 0; i < vs.size(); ++i)
	{
		vs[i]->g_score = LARGE_VALUE;
		vs[i]->f_score = LARGE_VALUE;
	}
	start->g_score = 0;
	start->f_score = heuristic_distance(start, end);

	while (open_set.size() > 0)
	{
		vertex *current = *std::min_element(open_set.begin(), open_set.end(), f_lessthan());	// lowest f_scored node
		std::cout << "Selected vertex with f score " << current->f_score << "\n";
		if (current == end) {
			std::cout << "Reached end vertex\n";
			return reconstruct_path(start, end);
		}
		open_set.erase(current);
		closed_set.insert(current);
//		std::cout << current->edges.size() << " neighbours.\n";
		for (size_t i = 0; i < current->edges.size(); ++i)
		{
			edge *e = current->edges[i];
			vertex *neighbour = e->other(current);
			//std::cout << "Considering vertex at (" << neighbour->posx << ", " << neighbour->posy << ")\n";
			if (closed_set.find(neighbour) != closed_set.end())
			{
				// if we've already fully determined this node, skip
				//std::cout << "Already closed set.\n";
				continue;
			}
			float tentative_g_score = current->g_score + e->length;
			if (open_set.find(neighbour) == open_set.end())
			{
				//std::cout << "Adding to open set\n";
				open_set.insert(neighbour);
			}
			else if (tentative_g_score >= neighbour->g_score)
			{
				//std::cout << "Already in open set, score not improved upon\n";
				// we have already considered this node and found a path as good
				// or better
				continue;
			}
			//std::cout << "Best score so far for this node.\n";
			// Otherwise this is the best path found so far for this node
			neighbour->came_from = e;
			neighbour->g_score = tentative_g_score;
			neighbour->f_score = tentative_g_score + heuristic_distance(neighbour, end);
		}
	}

	std::cout << "Pathfinding failed!\n";
	return std::vector<edge*>(); // Failed!
}

float world_map::angle_between(float ax, float ay, float bx, float by)
{
	float dot = ax * bx + ay * by;
	float cross = ay * bx - ax * by;
	return atan2(cross, dot) * 180.f / M_PI;
}

float world_map::turning_angle(edge *leaving, edge *entering, vertex *currently_at)
{
	return angle_between(leaving->dirx(currently_at),
						leaving->diry(currently_at),
						-entering->dirx(currently_at),
						-entering->diry(currently_at));
}

float world_map::turning_angle(float dirx, float diry, edge *entering, vertex *currently_at)
{
	return angle_between(
			dirx,
			diry,
			-entering->dirx(currently_at),
			-entering->diry(currently_at));
}



