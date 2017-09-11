#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>

#include "HybridAStar.h"
#include "State.h"
#include "Path.h"
#include "../Helpers.h"

using namespace std;

/**
* Initializes HybridAStar
*/
HybridAStar::HybridAStar() { }

HybridAStar::~HybridAStar() { }

void HybridAStar::init(double steeringLow, double steeringHigh, double steeringDelta, double speed) {
	this->SPEED = speed;
	this->STEER_LOW = steeringLow;
	this->STEER_HIGH = steeringHigh;
	this->THETA_DELTA = steeringDelta;
}


int HybridAStar::theta_to_stack_number(double theta) {
	/*
	Takes an angle (in radians) and returns which "stack" in the 3D configuration space
	this angle corresponds to. Angles near 0 go in the lower stacks while angles near
	2 * pi go in the higher stacks.
	*/

	double new_theta = fmod((theta + 2 * PI), (2 * PI));
	int stack_number = (int)(round(new_theta * NUM_THETA_CELLS / (2 * PI))) % NUM_THETA_CELLS;
	return stack_number;
}


int HybridAStar::idx(double float_num) {
	/*
	Returns the index into the grid for continuous position. So if x is 3.621, then this
	would return 3 to indicate that 3.621 corresponds to array index 3.
	*/

	return int(floor(float_num));
}

double HybridAStar::heuristic(State state, State stateP, State goal) {
	double h = 0;
	// minimise steering
	h += std::sqrt(std::pow(state.theta - stateP.theta, 2.0));
	// minimise jerk
	h += std::sqrt(std::pow(state.x - stateP.x, 2.0) + std::pow(state.y - stateP.y, 2.0) + std::pow(state.theta - stateP.theta, 2.0));
	// minimise distance to goal
	h += std::sqrt(std::pow(stateP.x - goal.x, 2.0) + std::pow(stateP.y - goal.y, 2.0) + std::pow(stateP.theta - goal.theta, 2.0));

	return h;
}


vector<State> HybridAStar::expand(State state, State goal) {
	int g = state.g;
	double x = state.x;
	double y = state.y;
	double theta = state.theta;
	double h = state.h;

	int g2 = g + 1;
	vector<State> next_states;
	for (double delta_i = this->STEER_LOW; delta_i <= STEER_HIGH; delta_i += THETA_DELTA)
	{

		double delta = PI / 180.0 * delta_i;
		double omega = SPEED / LENGTH * tan(delta);
		double theta2 = theta + omega;
		if (theta2 > 0)
		{
			theta2 += 2 * PI;
		}
		double x2 = x + SPEED * cos(theta2);
		double y2 = y + SPEED * sin(theta2);
		double h2 = h;

		State state2;
		state2.g = g2;
		state2.x = x2;
		state2.y = y2;
		state2.h = heuristic(state, state2, goal);
		state2.theta = theta2;

		next_states.push_back(state2);

	}
	return next_states;
}

vector<State> HybridAStar::reconstruct_path(vector<vector<vector<State>>> prev_states, State start, State final) {

	vector<State> path = { final };

	int stack = theta_to_stack_number(final.theta);

	State current = prev_states[stack][idx(final.x)][idx(final.y)];

	stack = theta_to_stack_number(current.theta);

	double x = current.x;
	double y = current.y;
	while (x != start.x && y != start.y)
	{
		path.push_back(current);
		current = prev_states[stack][idx(x)][idx(y)];
		x = current.x;
		y = current.y;
		stack = theta_to_stack_number(current.theta);
	}

	return path;
}

Path HybridAStar::search(vector<vector<int>> grid, State start, State goal) {
	
	vector<vector<vector<State>>> closed(NUM_THETA_CELLS, vector<vector<State>>(grid[0].size(), vector<State>(grid.size())));
	vector<vector<vector<int>>> closed_value(NUM_THETA_CELLS, vector<vector<int>>(grid[0].size(), vector<int>(grid.size())));
	vector<vector<vector<State>>> prev_states(NUM_THETA_CELLS, vector<vector<State>>(grid[0].size(), vector<State>(grid.size())));
	
	double theta = start.theta;
	int stack = theta_to_stack_number(theta);
	int g = 0;

	State state;
	state.g = g;
	state.x = start.x;
	state.y = start.y;
	state.h = heuristic(state, state, goal);

	closed[stack][idx(state.x)][idx(state.y)] = state;
	closed_value[stack][idx(state.x)][idx(state.y)] = 1;
	prev_states[stack][idx(state.x)][idx(state.y)] = state;
	
	int total_closed = 1;
	vector<State> opened = { state };
	
	bool finished = false;

	while (!opened.empty())
	{
		std::sort(opened.begin(), opened.end());
		State next = opened[0]; //grab first elment
		opened.erase(opened.begin()); //pop first element

		int x = next.x;
		int y = next.y;

		if (idx(x) == goal.x && idx(y) == goal.y)
		{
			cout << "found path to goal in " << total_closed << " expansions" << endl;
			
			Path path;
			path.found = true;
			path.points = reconstruct_path(prev_states, state, goal);
			
			return path;
		}
		vector<State> next_state = expand(next, goal);

		for (int i = 0; i < next_state.size(); i++)
		{
			int g2 = next_state[i].g;
			double x2 = next_state[i].x;
			double y2 = next_state[i].y;
			double theta2 = next_state[i].theta;
			double h2 = next_state[i].h;

			if ((x2 < 0 || x2 >= grid.size()) || (y2 < 0 || y2 >= grid[0].size()))
			{
				continue;
			}

			int stack2 = theta_to_stack_number(theta2);

			if (closed_value[stack2][idx(x2)][idx(y2)] == 0 && grid[idx(x2)][idx(y2)] == 0)
			{

				State state2;
				state2.g = g2;
				state2.x = x2;
				state2.y = y2;
				state2.theta = theta2;
				state2.h = h2;

				opened.push_back(state2);

				closed[stack2][idx(x2)][idx(y2)] = next_state[i];
				closed_value[stack2][idx(x2)][idx(y2)] = 1;
				prev_states[stack2][idx(x2)][idx(y2)] = next;
				total_closed += 1;
			}
		}
	}

	cout << "no valid path." << endl;
	Path path;

	return path;
}

