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
#define VERT_ABOVE_EGGS 7
#define VERT_45_NEAR_EGG_BOX 4
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
	bool ignore_junction; // If we will not detect this junction due to its geometry

	void (*endpoint_task)(robot_state &state);
	vertex(float posx_, float posy_, bool ignore_junction_ = false) {posx = posx_; posy = posy_; ignore_junction = ignore_junction_;}
};

struct edge {
	vertex *a, *b;
	float length;
	// the direction of the edge (towards vertex) at a, b
	// so that we know what angle to turn through at junctions
	float dir_ax, dir_ay, dir_bx, dir_by;
	// call as v->edge->other(v) to explore neighbours of v
	vertex* other(vertex *v) {return v == a ? b : a;}
	float dirx(vertex *v) {return v == a ? dir_ax : dir_bx;}
	float diry(vertex *v) {return v == a ? dir_ay : dir_by;}
	edge(vertex *a_, vertex *b_, float length = -1.f, float dir_ax_ = 0, float dir_ay_ = 0, float dir_bx_ = 0, float dir_by_ = 0);
};

struct world_map {
	const static float LARGE_VALUE = 1E+9;
	std::vector<vertex*> vs;
	std::vector<edge*> es;

	std::vector<edge*> reconstruct_path(vertex *start, vertex *end);
	std::vector<edge*> find_path(vertex *start, vertex *end);
	// Returns angle in degrees to be turned at the junction (currently_at)
	// between two vertices.
	float angle_between(float ax, float ay, float bx, float by);
	float turning_angle(edge *leaving, edge *entering, vertex *currently_at);
	float turning_angle(float dirx, float diry, edge *entering, vertex *currently_at);

	world_map();
	world_map(std::vector<vertex*> vs_, std::vector<edge*> es_) {vs = vs_; es = es_;}
};

#endif // _NAVIGATION_H_
