#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include <vector>
#include "robot_state.h"

#define VERT_EGG_0 17
#define VERT_EGG_1 12
#define VERT_EGG_2 11
#define VERT_EGG_3 10
#define VERT_EGG_4 9
#define VERT_EGG_BOX 0
#define VERT_CHICK_BOX 1
#define VERT_FRYING_PAN 2
#define VERT_START 14

struct edge;

struct vertex {
	float posx, posy;
	std::vector<edge*> edges;

	// Pathfinding fields:
	float g_score, f_score;	// g: cost of reaching node, f: heuristic total cost
	edge *came_from;

	void (*endpoint_task)(robot_state &state);
	vertex(float posx_, float posy_) {posx = posx_; posy = posy_;}
};

struct edge {
	vertex *a, *b;
	float length;
	// call as v->edge->other(v) to explore neighbours of v
	vertex* other(vertex *v) {return v == a ? b : a;}
	edge(vertex *a_, vertex *b_, float length = -1.f);
};

struct world_map {
	const static float LARGE_VALUE = 1E+9;
	std::vector<vertex*> vs;
	std::vector<edge*> es;

	std::vector<edge*> reconstruct_path(vertex *start, vertex *end);
	std::vector<edge*> find_path(vertex *start, vertex *end);

	world_map();
	world_map(std::vector<vertex*> vs_, std::vector<edge*> es_) {vs = vs_; es = es_;}
};

#endif // _NAVIGATION_H_
