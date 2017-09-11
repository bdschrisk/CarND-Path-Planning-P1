#ifndef CAR_H_
#define CAR_H_

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

	StateBase realise(double t) const {

		StateBase state;
		state.x = this->x + (this->v * t * sin(this->theta));
		state.y = this->y + (this->v * t * cos(this->theta));
		
		return state;
	};
};

#endif