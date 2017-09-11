#include "Model.h"

#include <algorithm>
#include <vector>
#include <cmath>
#include "../spline.h"

#include "../Helpers.h"

using namespace tk;
using namespace std;

const double MAX_SENSE_DIST = 100; // 100m
const double MAX_SPEED = 22.35; // 50 mph in m/s
const double SAFETY_DIST_FACTOR = 2.4; // number of car lengths
const double CYCLES_PS = 0.02; // 1 / cycles per second (50)
const double PLAN_DIST = 30.0; // 30m planning distance
const bool FORWARD_COST = true; // ahead vehicles only in costs

const int MAX_PATH_LENGTH = 50; // The max path length for trajectory planning
const int SMOOTH_WINDOW = 4; // state smoother

Model::Model() { }

Model::~Model() { }

/// HELPERS ///

/* Computes the best velocity given the state of the vehicle. */
static double getVelocity(const Lane current, const Lane target, double sref, double dref, double v) {

	auto minvel = [](Lane lane, double sval) {
		double vtarget = 0; double sclose = MAX_SENSE_DIST;
		for (unsigned int i = 0; i < lane.vehicles.size(); i++) {
			if (lane.vehicles[i].s <= sclose && lane.vehicles[i].s >= sval) {
				sclose = lane.vehicles[i].s;
				vtarget = lane.vehicles[i].v;
			}
		}
		return vtarget;
	};

	double fmin1 = minvel(current, sref);
	double fmin2 = minvel(target, sref);

	double weight1 = distance(current.center, dref);
	double weight2 = distance(target.center, dref);
	double sumw = (weight1 + weight2);

	weight1 = ((1.0 - weight1) / sumw);
	weight2 = ((1.0 - weight2) / sumw);

	return (fmin1 * weight1 + fmin2 * weight2 + v) / 2.0;
}

/* Returns the vehicles in the given lane. */
static vector<Car> vehiclesInLane(const int lane, const vector<Car> vehicles, const double sref, const bool forward) {
	vector<Car> results;

	for (unsigned int j = 0; j < vehicles.size(); j++) {
		if ((forward && vehicles[j].s >= (sref - PLAN_DIST * 0.5)) || !forward) {
			Car vehicle = vehicles[j];

			if (vehicle.lane == lane)
			{
				results.push_back(vehicle);
			}
		}
	}

	return results;
}

/* Returns the min, mean and max triple for a given lane. */
static vector<double> laneSpeeds(const vector<Car> vehiclesInLane) {
	double mean_ = MAX_SPEED, min_ = MAX_SPEED, max_ = MAX_SPEED;

	for (unsigned int i = 0; i < vehiclesInLane.size(); i++) {
		mean_ += vehiclesInLane[i].v;
		min_ = min(min_, vehiclesInLane[i].v);
		max_ = max(max_, vehiclesInLane[i].v);
	}

	mean_ /= (vehiclesInLane.size() + 1);

	return { min_, mean_, max_ };
}

/* Performs lane availability checks at a given timeframe. */
static bool laneCheck(const Lane current, const Lane target, const Car ego, const double t,
	const WorldMap environment) {
	bool valid = false;
	
	// check for adjacent lane
	if (abs(target.id - current.id) <= 1) {
		if (current.id != target.id) {
			// check collisions...
			valid = true;
			StateBase egocheck = ego.realise(t);

			for (unsigned int i = 0; i < target.vehicles.size(); i++) {
				StateBase check = target.vehicles[i].realise(t);

				vector<double> egof = environment.getFrenet(egocheck.x, egocheck.y, ego.theta);
				vector<double> carf = environment.getFrenet(check.x, check.y, target.vehicles[i].theta);

				valid &= collides(carf[0], carf[1], target.vehicles[i].width, target.vehicles[i].length,
					egof[0], egof[1], ego.width, ego.length) == false;

				double dist = (distance(egof[1], carf[1]) - (ego.length * 0.5 + target.vehicles[i].length * 0.5));
				valid &= dist >= (SAFETY_DIST_FACTOR * ego.length);
			}
		}
	}

	return valid;
}

/* Returns a lane object given the current world state */
static Lane getLane(const int id, const double sref, const vector<Car> vehicles, const bool aheadonly, 
	const WorldMap environment) {
	Lane lane;

	auto cars = vehiclesInLane(id, vehicles, sref, aheadonly);
	auto stats = laneSpeeds(cars);

	lane.id = id;
	lane.width = environment.laneWidth(id, sref);
	lane.min_velocity = stats[0];
	lane.mean_velocity = stats[1];
	lane.max_velocity = stats[2];
	lane.vehicles = cars;
	lane.center = environment.laneCenter(id, sref);

	return lane;
}

/// END HELPERS ///

/* Searches for a lane using value iteration to find the optimal lane. */
Lane Model::optimalLane() {

	auto policy = [](double minv, double muv, double maxv, int vehicles) {
		double r = ((minv + muv + maxv) / 3.0);
		double c = 1.0 - ((Car::M_CAR_LENGTH / MAX_SENSE_DIST) * vehicles);

		return (r * 0.6 + c * 0.4);
	};
	
	double best_cost = 0; Lane best_lane;
	auto lanes = this->environment.lanes(this->ego.s);
	for (unsigned int id = 0; id < lanes; id++) {
		// get lane
		Lane lane = getLane(id, this->ego.s, this->vehicles, FORWARD_COST, this->environment);
		// compute lane policy
		double v = policy(lane.min_velocity, lane.mean_velocity, lane.max_velocity, lane.vehicles.size());

		if (v > best_cost) {
			best_cost = v;
			best_lane = lane;
		}

		return best_lane;
	}
}

/* Updates the ego state using motion prediction */
void Model::predict(double t) {
	StateBase stateP = this->ego.realise(t);
	this->ego.x = stateP.x;
	this->ego.y = stateP.y;
	vector<double> frenet = this->environment.getFrenet(stateP.x, stateP.y, this->ego.theta);
	this->ego.s = frenet[0]; this->ego.d = frenet[1];

	this->ego.lane = this->environment.whichLane(this->ego.s, this->ego.d);
}

void Model::update(Car ego, vector<vector<int>> sensorFusion) {

	this->vehicles.clear();

	// update state of the model
	this->ego.d = ego.d;
	this->ego.s = ego.s;
	this->ego.theta = ego.theta;
	this->ego.v = ego.v;
	this->ego.x = ego.x;
	this->ego.y = ego.y;
	this->ego.lane = this->environment.whichLane(ego.s, ego.d);
	// predict new ego state
	this->predict(CYCLES_PS);

	for (unsigned int i = 0; i < sensorFusion.size(); i++) {
		Car car;

		int id = sensorFusion[i][0];

		car.x = sensorFusion[i][1];
		car.y = sensorFusion[i][2];
		double vx = sensorFusion[i][3];
		double vy = sensorFusion[i][4];

		car.theta = atan2(vy, vx);
		car.s = sensorFusion[i][5];
		car.d = sensorFusion[i][6];

		car.v = metric_speed(velocity(vx, vy));

		car.lane = this->environment.whichLane(car.s, car.d);
		car.distance = distance(this->ego.x, this->ego.y, car.x, car.y);

		if (car.distance <= MAX_SENSE_DIST) {
			this->vehicles.push_back(car);
		}
	}

	if (this->currentlane.id < 0) {
		int id = this->environment.whichLane(this->ego.s, this->ego.d);
		this->currentlane = getLane(id, this->ego.s, this->vehicles, true, this->environment);
	}
}

Path Model::plan(double max_speed, vector<double> previous_xpath, vector<double> previous_ypath) {

	int prev_size = previous_xpath.size();
	vector<double> prev_x, prev_y;
	double targetv = this->ego.v;

	double ref_yaw = this->ego.theta;
	double ref_x = this->ego.x, ref_y = this->ego.y;

	// append previous waypoints to solver
	if (prev_size < SMOOTH_WINDOW) {
		// use tangent path on cold start
		double prev_ego_x = ego.x - cos(ego.theta);
		double prev_ego_y = ego.y - sin(ego.theta);

		prev_x.push_back(prev_ego_x); prev_x.push_back(ego.x);
		prev_y.push_back(prev_ego_y); prev_y.push_back(ego.y);
	}
	else {
		for (int i = prev_size - (SMOOTH_WINDOW - 1); i < prev_size; i++) {
			ref_x = previous_xpath[i];
			ref_y = previous_ypath[i];
			
			prev_x.push_back(ref_x);
			prev_y.push_back(ref_y);

			double ref_px = previous_xpath[i - 1];
			double ref_py = previous_ypath[i - 1];
			ref_yaw = atan2(ref_y - ref_py, ref_x - ref_py);
		}
	}

	Path solverpath;

	Lane optimal = this->optimalLane();
	if (this->ego.lane == optimal.id) {
		// keep lane state
		
		// define target velocity
		targetv = getVelocity(this->currentlane, optimal, this->ego.s, this->ego.d, this->ego.v);

		solverpath.found = true;

		// accumulate more points to follow for another set distance (100m)
		for (int dist = PLAN_DIST; dist < MAX_SENSE_DIST; dist += PLAN_DIST) {
			double sref = this->ego.s + dist;
			double dref = this->environment.laneCenter(this->ego.lane, sref);
			
			vector<double> waypoint = this->environment.getXY(sref, dref);
			
			State state(waypoint[0], waypoint[1], 0);
			solverpath.points.push_back(state);
		}

		solverpath.localisePoints(ref_x, ref_y, (0.0 - ref_yaw));
	}
	else {
		// if need intermediary changes cycle through lanes
		Lane targetlane = optimal;

		auto nextlane = [](int current, int target) {
			return current + (target < current ? -1 : 1);
		};

		if (abs(optimal.id - ego.lane) > 1) {
			int translane = nextlane(this->currentlane.id, optimal.id);
			targetlane = getLane(translane, this->ego.s, this->vehicles, FORWARD_COST, this->environment);
		}

		solverpath.found = true;
		// accumulate more points to follow for another set distance (100m)
		double dist_acc = ((PLAN_DIST / SAFETY_DIST_FACTOR) / this->ego.length);
		for (int dist = PLAN_DIST; dist < MAX_SENSE_DIST; dist += dist_acc) {
			
			double delta_t = (dist / (CYCLES_PS * targetv));
			// check for lane merge possibility
			if (laneCheck(this->currentlane, targetlane, this->ego, delta_t, this->environment)) {
				
				double sref = this->ego.s + dist;
				double dref = this->environment.laneCenter(targetlane.id, sref);

				vector<double> waypoint = this->environment.getXY(sref, dref);

				State state(waypoint[0], waypoint[1], 0);
				solverpath.points.push_back(state);

				if (this->environment.whichLane(sref, dref) != this->currentlane.id) {
					this->currentlane = targetlane;
					
					int id = nextlane(this->currentlane.id, optimal.id);
					targetlane = getLane(id, sref, this->vehicles, FORWARD_COST, this->environment);
				}
			}
		}
	}

	Path solution;
	// assign previous path to solution
	for (unsigned int i = 1; i < previous_xpath.size(); i++) {
		solution.addWaypoint(previous_xpath[i], previous_ypath[i]);
	}

	spline s;
	s.set_points(solverpath.xPoints(), solverpath.yPoints());

	double target_x = PLAN_DIST;
	double target_y = s(target_x);
	double target_dist = distance(target_x, target_y);

	double x_add = 0;
	// generate horizon waypoints
	for (int i = 1; i <= MAX_PATH_LENGTH - prev_size; i++) {

		double N = (target_dist / (CYCLES_PS * targetv));
		double xp = x_add + (target_x / N);
		double yp = s(xp);

		x_add = xp;

		double xpp = translateGlobalX(xp, yp, ref_yaw, ref_x, ref_y);
		double ypp = translateGlobalY(xp, yp, ref_yaw, ref_x, ref_y);

		solution.addWaypoint(xpp, ypp);
	}

	return solution;
}