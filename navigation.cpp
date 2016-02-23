#include "navigation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>

const float root2 = 1.414;

edge::edge(vertex *a_, vertex *b_, float length)
{
	if (a_ == NULL || b_ == NULL)
		return;	// should throw
	a = a_;
	b = b_;
	a->edges.push_back(this);
	b->edges.push_back(this);
	// calculate length if no length specified:
	// (must be specified if path not straight)
	if (length < 0)
		length = sqrtf(powf(b->posx - a->posx, 2) + powf(b->posy - a->posy, 2));
}

world_map::world_map()
{
	vs.push_back(new vertex(600 * (1 - root2/2), 1200 + 600 * root2/2));
	vs.push_back(new vertex(900, 1800));
	vs.push_back(new vertex(1200 + 600 * root2/2, 1200 + 600 * root2/2));
	vs.push_back(new vertex(0, 1200));
	vs.push_back(new vertex(600, 1200));
	vs.push_back(new vertex(900, 1200));
	vs.push_back(new vertex(1200, 1200));
	vs.push_back(new vertex(1800, 1200));
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

	es.push_back(new edge(vs[0], vs[1]));
	es.push_back(new edge(vs[0], vs[3]));
	es.push_back(new edge(vs[0], vs[4]));
	es.push_back(new edge(vs[1], vs[2]));
	es.push_back(new edge(vs[1], vs[5]));
	es.push_back(new edge(vs[2], vs[6]));
	es.push_back(new edge(vs[2], vs[7]));
	es.push_back(new edge(vs[3], vs[4]));
	es.push_back(new edge(vs[3], vs[13]));
	es.push_back(new edge(vs[4], vs[5]));
	es.push_back(new edge(vs[5], vs[6]));
	es.push_back(new edge(vs[5], vs[8]));
	es.push_back(new edge(vs[6], vs[7]));
	es.push_back(new edge(vs[7], vs[9]));
	es.push_back(new edge(vs[8], vs[9]));
	es.push_back(new edge(vs[8], vs[16]));
	es.push_back(new edge(vs[9], vs[10]));
	es.push_back(new edge(vs[10], vs[11]));
	es.push_back(new edge(vs[11], vs[12]));
	es.push_back(new edge(vs[12], vs[17]));
	es.push_back(new edge(vs[13], vs[14]));
	es.push_back(new edge(vs[14], vs[15]));
	es.push_back(new edge(vs[15], vs[16]));
	es.push_back(new edge(vs[16], vs[17]));
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

	while (open_set.size() > 0)
	{
		vertex *current = *std::min_element(open_set.begin(), open_set.end(), f_lessthan());	// lowest f_scored node
		if (current == end) {
			std::cout << "Reached end vertex\n";
			return reconstruct_path(start, end);
		}
		open_set.erase(current);
		closed_set.insert(current);
		std::cout << current->edges.size() << " neighbours.\n";
		for (size_t i = 0; i < current->edges.size(); ++i)
		{
			std::cout << "Considering vertex\n";
			edge *e = current->edges[i];
			vertex *neighbour = e->other(current);
			if (closed_set.find(neighbour) != closed_set.end())
			{
				// if we've already fully determined this node, skip
				std::cout << "Already closed set.\n";
				continue;
			}
			float tentative_g_score = current->g_score + e->length;
			if (open_set.find(neighbour) == open_set.end())
			{
				std::cout << "Adding to open set\n";
				open_set.insert(neighbour);
			}
			else if (tentative_g_score >= neighbour->g_score)
			{
				std::cout << "Already in open set, score not improved upon\n";
				// we have already considered this node and found a path as good
				// or better
				continue;
			}
			std::cout << "Best score so far for this node.\n";
			// Otherwise this is the best path found so far for this node
			neighbour->came_from = e;
			neighbour->g_score = tentative_g_score;
			neighbour->f_score = tentative_g_score + heuristic_distance(neighbour, end);
		}
	}

	std::cout << "Pathfinding failed!\n";
	return std::vector<edge*>(); // Failed!
}