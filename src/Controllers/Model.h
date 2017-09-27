#ifndef MODEL_H_
#define MODEL_H_

#include <vector>
#include "../spline.h"
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
	int iter;
	double mean_off;
	// previous path
	Path path;
	// target lane of ego
	int currentlane;
	// s value of last lane shift
	double lanechange_s;
	// initialised status
	bool _initialized;

	bool _replan;

	double currentvel;
	tk::spline acc_curve;
	tk::spline brake_curve;

	vector<Lane> lanes;
	vector<double> velocities;

	/* Finds the lane that maximises the distance over time */
	Lane optimalLane();

	bool canLaneChange(double s);

	/* Updates the ego state using motion prediction */
	void predict(double t);

	double getOptimalVelocity(const Lane current, const Lane target, double sref, double dref, double v);
	double getVelocity(double vi, double vf, double t);

public:

	// environment map
	WorldMap environment;
	// planner
	HybridAStar solver;
	// car
	Car ego;

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
	void update(double max_speed, Car egostate, vector<vector<int>> sensorFusion);

	/**
	* Returns the planned trajectory given a solver path.
	*/
	Path trajectory(Path solverpath, double max_t, int m_offset, double ref_vel, double targetv, StateBase ref_state);

	/* Plans the new trajectory given the previously consumed path points */
	Path plan(double sref, double dref, vector<double> previous_xpath, vector<double> previous_ypath);
};

#endif
