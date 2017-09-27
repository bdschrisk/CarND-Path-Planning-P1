#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

#include "Car.h"

using namespace std;

struct Lane {

	int id = -1;

	double static_max_v;

	double min_velocity;
	double mean_velocity;
	double max_velocity;

	double width;
	double center;
	double offset;

	vector<Car> vehicles;

	/* Updates the lane stats */
	void updateLane(double max_speed) {
		this->static_max_v = max_speed;
		double mean_ = max_speed, min_ = max_speed, max_ = max_speed;

		for (unsigned int i = 0; i < this->vehicles.size(); i++) {
			mean_ += this->vehicles[i].v;
			min_ = min(min_, this->vehicles[i].v);
			max_ = max(max_, this->vehicles[i].v);
		}
		this->min_velocity = min_;
		this->max_velocity = max_;
		this->mean_velocity /= max((double)this->vehicles.size(), 1.0);
	}

	vector<double> minVelocity(double max_sensor_dist, double safety_brake_factor, double egolength, double sref) const {
		double accel = 0;
		double min_gap = max_sensor_dist;
		double vtarget = this->max_velocity;
		double safety_distance = (safety_brake_factor * egolength);

		for (unsigned int i = 0; i < this->vehicles.size(); i++) {
			Car check = this->vehicles[i];

			double gap = (check.s - sref);
			double car_speed = check.v;

			cout << " DEBUG: vehicle " << i << " in front by " << gap << "m, travelling at " << car_speed << "m/s" 
				 << "  | s: " << check.s << " : " << sref << endl;
			if (gap > 0.0) {	
				cout << " DEBUG: tracking vehicle " << i << " at " << car_speed << "m/s" << endl;
				if ((gap < safety_distance) && (gap < min_gap)) {
					min_gap = gap;
					vtarget = car_speed;
				}
			}
		}

		return { vtarget, min_gap };
	}

	double cost(double max_dist) {
		double r = (this->max_velocity / this->static_max_v) * (this->mean_velocity / this->static_max_v) 
					* (this->min_velocity / this->static_max_v);
		
		double c = 1.0 - ((Car::M_CAR_LENGTH / max_dist) * (double)this->vehicles.size());

		return (r * c);
	}
};
