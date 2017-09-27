#ifndef CAR_H_
#define CAR_H_

#include <cmath>

#include "../Search/State.h"
#include "../Helpers.h"

using namespace std;

class Car : public State {

public:
	double s; // t0
	double d; // t0
	double v;
	int lane;
	double distance;

	double width;
	double length;

	static const int M_CAR_LENGTH = 4.5;
	static const int M_CAR_WIDTH = 2.7;

	Car() {
		this->width = M_CAR_WIDTH;
		this->length = M_CAR_LENGTH;
	}

	virtual ~Car() { }

	Car realise(double t) const {

		Car state;
		state.x = this->x + (this->v * t * sin(this->theta));
		state.y = this->y + (this->v * t * cos(this->theta));
		state.theta = atan2(this->y - state.y, this->x - state.x);
		
		state.s = this->s + (this->v * t);
		state.d = this->d;
		state.v = this->v;
		state.lane = this->lane;
		state.width = this->width;
		state.length = this->length;

		return state;
	};
};

#endif