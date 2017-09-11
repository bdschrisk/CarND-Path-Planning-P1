#ifndef HYBRID_A_STAR_H_
#define HYBRID_A_STAR_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <vector>

#include "State.h"
#include "Path.h"

using namespace std;

class HybridAStar {

public:

	int NUM_THETA_CELLS = 90;
	double SPEED = 1.45;
	double LENGTH = 0.5;
	double STEER_LOW = -35;
	double STEER_HIGH = 35;
	double THETA_DELTA = 5;

	/**
	* Constructor
	*/
	HybridAStar();

	/**
	* Destructor
	*/
	virtual ~HybridAStar();

	void init(double steeringLow, double steeringHigh, double steeringDelta, double speed);

	int theta_to_stack_number(double theta);

	int idx(double float_num);

	/**
	* Heuristic for transitioning from one state to another
	*/
	double heuristic(State state, State stateP, State goal);

	/**
	* Expands the state
	*/
	vector<State> expand(State state, State goal);

	/**
	* Searches the input stace for a valid path and returns one if it exists
	*/
	Path search(vector<vector<int>> grid, State start, State goal);

	/**
	* Reconstructs the valid path
	*/
	vector<State> reconstruct_path(vector<vector<vector<State>>> prev_states, State start, State final);


};

#endif
