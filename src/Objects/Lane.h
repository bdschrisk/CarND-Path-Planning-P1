#include <vector>

#include "Car.h"

using namespace std;

struct Lane {

	int id = -1;

	double min_velocity;
	double mean_velocity;
	double max_velocity;

	double width;
	double center;

	vector<Car> vehicles;
};
