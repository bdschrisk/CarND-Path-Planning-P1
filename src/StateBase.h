#ifndef STATE_BASE_H_
#define STATE_BASE_H_

#include <cmath>

using namespace std;

class StateBase {

public:
	double x;
	double y;
	double theta;

	StateBase() { };
	StateBase(double x, double y, double theta) {
		this->x = x;
		this->y = y;
		this->theta = theta;
	}

	virtual bool collides(StateBase other, double tolerance = 0.5) {

		bool result = (this->x < other.x + tolerance && this->x + tolerance > other.x);
		result &= (this->y < other.y + tolerance && this->y + tolerance > other.y);
		
		return result;
	}
};

#endif