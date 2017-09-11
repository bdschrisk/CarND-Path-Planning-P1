#ifndef MODEL_H_
#define MODEL_H_

#include <vector>
#include "../WorldMap.h"
#include "../Objects/Car.h"
#include "../Objects/Lane.h"
#include "../Search/HybridAStar.h"
#include "../Search/State.h"
#include "../Search/Path.h"
#include "JMT.h"

using namespace std;

class Model {

private:
	// previous path
	Path path;
	// target lane of ego
	Lane currentlane;
	/* Finds the lane that maximises the distance over time */
	Lane optimalLane();

	/* Updates the ego state using motion prediction */
	void predict(double t);

public:

	// environment map
	WorldMap environment;
	// planner
	HybridAStar solver;
	// car
	Car ego;
	// other vehicles
	vector<Car> vehicles;

	JMT jmt;

	/**
	* Constructor
	*/
	Model();

	/**
	* Destructor
	*/
	virtual ~Model();

	/**
	* Updates the model with the current state of the environment.
	*/
	void update(Car egostate, vector<vector<int>> sensorFusion);

	/* Plans the new trajectory given the previously consumed path points and required path length */
	Path plan(double max_speed, vector<double> previous_xpath, vector<double> previous_ypath);
};

#endif
