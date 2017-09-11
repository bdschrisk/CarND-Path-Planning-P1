#ifndef STATE_H_
#define STATE_H_

#include "../StateBase.h"

struct State : public StateBase {

	double theta;
	
	int g;
	double h;

	State() { }
	State(double x, double y, double theta, int g = 0, double h = 0) {
		this->x = x;
		this->y = y;
		this->theta = theta;

		this->g = g;
		this->h = h;
	}

	bool operator < (const State &state) const { return state.h > h; }

};

#endif