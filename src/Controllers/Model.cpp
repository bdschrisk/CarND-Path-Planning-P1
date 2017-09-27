#include "Model.h"

#include <algorithm>
#include <vector>
#include <cmath>
#include <iostream>

#include "../spline.h"
#include "../Helpers.h"

using namespace std;
using namespace tk;

const double CYCLES_PS = 0.02; // 1 / cycles per second (50)
const double PLAN_DIST = 30.0; // 30m planning distance
const double MAX_T = 1.5; // future prediction time window
const double WARM_UP_T = 10.0; // warm up in sec

const double MAX_FSENSE_DIST = 100; // 150m forward sense distance
const double MAX_BSENSE_DIST = 40; // 50m rear sense distance
const double MAX_ACTION_DIST = (MAX_FSENSE_DIST - MAX_BSENSE_DIST); // lane cost distance

//const double MAX_LANE_T = 3.0; // max time for lane shift
const double MAX_LANE_T = 2.4; // max time for lane shift
const double MAX_SPEED = 22.35; // 50 mph in m/s
const double BUFFER_SPEED = 0.5; // 1.24 mph in m/s
const double MAX_T_SPEED = 10.0; // 10 second acceleration to top speed
const double FULL_ACCEL_T = (MAX_SPEED / MAX_T_SPEED); // standing start to MAX_SPEED in sec^2
const double MAX_A_V = 0.2262; // max change in velocity

const double SAFETY_DIST_FACTOR = 3.2; // number of car lengths
const double SAFETY_BRAKE_FACTOR = 4.5; // safe braking distance
const double SAFETY_OVERTAKE_DIST = 45.0; // safe overtaking distance
const double SAFETY_OVERTAKE_FACTOR_F = 2.8; // forward safety factor for overtaking
const double SAFETY_OVERTAKE_FACTOR_B = 2.2; // rear safety factor for overtaking

const int MIN_PATH_LENGTH = 2;
const int SMOOTH_WINDOW = 3; // state smoother

Model::Model() {
	this->currentlane = -1; // not set.
	
	vector<double> speed = { 0.0, MAX_SPEED - BUFFER_SPEED, MAX_SPEED, MAX_SPEED + BUFFER_SPEED, MAX_SPEED + 10 };
	vector<double> acc = { FULL_ACCEL_T, 0.0, 0.0, -0.01, -MAX_A_V };
	this->acc_curve.set_points(speed, acc, false);

	this->currentvel = 0;
	this->iter = 0;
	this->_initialized = false;
	this->lanechange_s = -1;
	this->mean_off = 0;
	this->_replan = true;
}

Model::~Model() { }

/// HELPERS ///

/* Performs lane availability checks at a given timeframe. */
static bool laneCheck(const Lane current, const Lane target, const Car &ego, const double t, const WorldMap environment) {
	bool valid = true;
	
	// check for adjacent lane
	if (abs(target.id - current.id) <= 1) {
		// check collisions...
		Car egocheck = ego.realise(t);

		double dref = target.offset;

		double safety_distance = (SAFETY_DIST_FACTOR * ego.length);

		for (unsigned int i = 0; i < target.vehicles.size(); i++) {
			Car check = target.vehicles[i].realise(t);

			bool collision = collides(check.s, dref, check.width, check.length,
				egocheck.s, dref, ego.width, ego.length);

			double dist = (abs_distance(egocheck.s, check.s));
			bool safe = (dist >= safety_distance && !collision);

			cout << "DEBUG [Collision]: Vehicle " << i << "  s = " << check.s << " | "
					<< egocheck.s << " Ego " << "| dx = " << dist << "m  | "
					<< (!safe ? " *Collision detected " : " ") << " time = " << t << "s" << endl;
			
			valid = (valid && safe);
		}
	}

	return valid;
}

/// END HELPERS ///

/* Computes the best velocity given the state of the vehicle. */
double Model::getOptimalVelocity(const Lane current, const Lane target, double sref, double dref, double v) {
	
	double vf = 0;

	if (current.id == target.id) {
		// adjust speed to match front vehicle within safety distance factor
		auto track = current.minVelocity(MAX_FSENSE_DIST, SAFETY_BRAKE_FACTOR, this->ego.length, sref);
		vf = track[0]; double disp = min(track[1], (SAFETY_DIST_FACTOR * this->ego.length));
		double a = accelDisplacement(v, vf, disp);
		double t = velocityTime(v, vf, a);
		cout << " DEBUG: time to acceleration hold: " << t << "sec" << endl;
		if (t < MAX_T && abs_distance(v, vf) > BUFFER_SPEED * 2 && v > vf) {
			cout << " DEBUG: applying brake curve: { " << vf << " --> " << v << " --> " << v + 0.001 << " }" << endl;

			vector<double> vx = { vf, v, v + 0.001 };
			vector<double> vy = { 0, a, a - 0.001 };
			this->brake_curve.set_points(vx, vy);
		}

		cout << " DEBUG: tracking velocity: " << vf << "m/s" << endl;
	}
	else {
		auto track_current = current.minVelocity(MAX_FSENSE_DIST, SAFETY_BRAKE_FACTOR, this->ego.length, sref);
		auto track_target = target.minVelocity(MAX_FSENSE_DIST, SAFETY_BRAKE_FACTOR, this->ego.length, sref);

		double weight1 = distance(current.center, dref);
		double weight2 = distance(target.center, dref);
		double sumw = (weight1 + weight2);

		weight1 = (1.0 - (weight1 / sumw));
		weight2 = (1.0 - (weight2 / sumw));
		cout << " DEBUG: current= " << track_current[0] << ", w1= " << weight1 << ", target= " << track_target[0] << ", w2= " << weight2 << endl;
		vf = (track_current[0] * weight1 + track_target[0] * weight2 + v) / 2.0;
		cout << " DEBUG: Cross velocity optimal: " << vf << "m/s" << endl;
	}

	return vf;
}

double Model::getVelocity(double vi, double vf, double t) {
	// adjust acceleration
	double accel = 0;
	double t0 = 0;
	double v = vi;

	while (t0 < t) {
		if (v < (vf - BUFFER_SPEED) || abs_distance(vi, vf) <= BUFFER_SPEED) {
			accel = rescale(this->acc_curve(v), 0, FULL_ACCEL_T, 0, (vf / 10.0));
		}
		else {
			//accel = ((vf - vi) / (PLAN_DIST / vi));
			accel = this->brake_curve(v);

			cout << "DEBUG [Brake]: Braking from " << vi << "m/s to " << vf << "m/s" << " | accel =" << accel << endl;
		}

		v = v + (accel * CYCLES_PS);
		t0 += CYCLES_PS;
	}

	return v;
}

/* Searches for a lane using value iteration to find the optimal lane. */
Lane Model::optimalLane() {

	Lane best_lane = this->lanes[this->currentlane];
	double best_cost = 0;
	double cost_sum = 0;
	double min_best = 0.48;

	vector<double> weights;
	
	auto lanes = this->environment.lanes(this->ego.s);
	for (unsigned int id = 0; id < lanes; id++) {
		// get lane
		Lane lane = this->lanes[id];
		// compute lane policy
		double v = lane.cost(MAX_ACTION_DIST);

		weights.push_back(v);
		cost_sum += v;
	}

	for (unsigned int i = 0; i < lanes; i++) {
		double v = weights[i] / cost_sum;

		if (v > min_best && v > best_cost) {
			best_lane = this->lanes[i];
			best_cost = v;
		}
		cout << "DEBUG [Lane]: " << i << ", cost = " << v << ", optimal = " << best_lane.id << endl;
	}

	return best_lane;
}

/* Updates the ego state using motion prediction */
void Model::predict(double t) {
	this->ego = this->ego.realise(t * this->mean_off);
}

bool Model::canLaneChange(double s) {
	//return !(((s - this->lanechange_s) < (SAFETY_OVERTAKE_DIST * 2)) && this->lanechange_s > 0);
	return (((s - this->lanechange_s) >= (SAFETY_OVERTAKE_DIST * 2)) || this->lanechange_s < 0);
}

void Model::update(double max_speed, Car egostate, vector<vector<int>> sensorFusion) {

	// update state of the model
	this->ego.d = egostate.d;
	this->ego.s = egostate.s;

	this->ego.theta = egostate.theta;

	this->ego.v = egostate.v;
	this->ego.x = egostate.x;
	this->ego.y = egostate.y;
	this->ego.lane = this->environment.whichLane(egostate.s, egostate.d);

	// update lanes
	this->lanes.clear();

	int numlanes = this->environment.lanes(this->ego.s);
	for (unsigned int lane = 0; lane < numlanes; lane++) {
		Lane lane_obj;
		lane_obj.id = lane;
		lane_obj.width = this->environment.laneWidth(lane, this->ego.s);
		lane_obj.center = this->environment.laneCenter(lane, this->ego.s);
		lane_obj.offset = this->environment.laneOffset(lane, this->ego.s);

		this->lanes.push_back(lane_obj);
	}

	// update vehicles
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

		car.v = (velocity(vx, vy));

		car.lane = this->environment.whichLane(car.s, car.d);
		car.distance = abs_distance(this->ego.s, car.s);
		bool ahead = (car.s > this->ego.s);

		if ((car.distance <= MAX_FSENSE_DIST && ahead) || (car.distance <= MAX_BSENSE_DIST && !ahead)) {
			this->lanes[car.lane].vehicles.push_back(car);
		}
	}

	cout << "Lanes: ";
	for (unsigned int lane = 0; lane < numlanes; lane++) {
		this->lanes[lane].updateLane(max_speed);
		cout << lane << " = " << this->lanes[lane].vehicles.size() << ", ";
	}
	cout << endl;

	if (this->_initialized == false) {
		this->currentlane = this->ego.lane;
		this->ego.v = MAX_A_V;
		this->_initialized = true;
	}
	else {
		//this->predict(CYCLES_PS * 0.5);
	}
	// DEBUG
	cout << "Ego state: { Lane: " << this->ego.lane << ", X: " << this->ego.x << ", Y: " 
		 << this->ego.y << ", Theta: " << this->ego.theta << ", S: " << this->ego.s << ", D: " << this->ego.d 
		 << ", V: " << this->ego.v << " } " << endl;
	cout << "Current Lane: { ID: " << this->currentlane << ", Vehicles: " << this->lanes[currentlane].vehicles.size() << " }" << endl;
}

Path Model::plan(double sref, double dref, vector<double> previous_xpath, vector<double> previous_ypath) {
	
	int prev_size = previous_xpath.size();
	int m_offset = (prev_size > 0 ? this->path.length() - prev_size : 0);
	double car_s = this->ego.s;
	double car_d = this->ego.d;
	double car_yaw = this->ego.theta;
	
	double ref_x = this->ego.x;
	double ref_y = this->ego.y;
	double ref_yaw = deg2rad(car_yaw);
	double ref_vel = this->ego.v;

	int lane = this->currentlane;
	double targetv = this->getOptimalVelocity(this->lanes[currentlane], this->lanes[currentlane], car_s, car_d, ref_vel);

	double max_s = car_s;

	bool cold_start = (this->iter <= (1.0 / CYCLES_PS) * WARM_UP_T);
	
	// DEBUG
	cout << "Path size: " << prev_size << ", offset: " << m_offset << endl;

	bool too_close = false;
	bool change_lane = false;

	Path solverpath;
	
	if (this->path.length() == 0) {
		// Use two points that make the path tangent to the car
		double prev_car_x = ref_x - cos(ref_yaw);
		double prev_car_y = ref_y - sin(ref_yaw);

		solverpath.addWaypoint(prev_car_x, prev_car_y);
		solverpath.addWaypoint(ref_x, ref_y);
	} 
	else {

		if (m_offset > 0) {
			double px = this->path.points[0].x;
			double py = this->path.points[0].y;
			solverpath.addWaypoint(px, py);

			for (unsigned int i = 1; i < m_offset; i++) {
				double x = this->path.points[i].x;
				double y = this->path.points[i].y;

				solverpath.addWaypoint(x, y);
				ref_yaw = atan2(y - py, x - px);

				px = x;
				py = y;
			}

			ref_vel = this->velocities[m_offset-1];
		}
		else {
			double prev_car_x = this->path.points[0].x;
			double prev_car_y = this->path.points[0].y;
			ref_yaw = atan2(ref_y - prev_car_y, ref_x - prev_car_x);
			solverpath.addWaypoint(prev_car_x, prev_car_y);
			solverpath.addWaypoint(ref_x, ref_y);
		}
	}

	this->predict(CYCLES_PS);
	
	// PLANNING //
	if (!cold_start) {
		Lane optimal_lane = optimalLane();
		change_lane = (abs(optimal_lane.id - this->currentlane) > 0);

		if (change_lane)
		{
			int lane_changes = 0;

			lane = optimal_lane.id;
			// check intermediate lane
			string change_state = (this->currentlane > optimal_lane.id ? "Left" : "Right");
			cout << "FSM: Prepare Lane Change " << change_state << " -> *" << lane << " via " << this->currentlane << endl;
			// gets the next lane
			auto nextlane = [](vector<Lane> lanes, Lane current, Lane optimal) {
				if (abs(optimal.id - current.id) > 0) {
					int translane = current.id + (optimal.id < current.id ? -1 : 1);
					return lanes[translane];
				}
				else {
					return current;
				}
			};

			// get intermediate lane
			Lane next_lane = nextlane(this->lanes, this->lanes[currentlane], optimal_lane);
			// safety buffer distance
			double min_t = (PLAN_DIST / ref_vel);
			//double min_t = 0.2;

			double tempv = ref_vel;
			double temptv = targetv;
			// look ahead in time if not safe now
			for (double t = CYCLES_PS; t <= MAX_LANE_T; t += CYCLES_PS) {				
				// check distance
				if (t > min_t) {
					
					double start_t = t;
					// check for lane merge possibility at time t
					if (laneCheck(this->lanes[currentlane], next_lane, this->ego, t, this->environment)) {
						Car egostate = this->ego.realise(t);
						double sp = egostate.s;

						if (this->canLaneChange(egostate.s)) {
							
							//double dp = this->environment.laneCenter(next_lane.id, sp);
							double ddelta = (this->currentlane > next_lane.id ? -1 : 1) 
												* (this->lanes[this->currentlane].width * 0.5);
							double dp = this->lanes[currentlane].center + ddelta;

							this->lanechange_s = sp;
							max_s = max(max_s, sp);

							vector<double> waypoint = this->environment.getXY(sp, dp);
							solverpath.addWaypoint(waypoint[0], waypoint[1]);

							change_state = (this->currentlane > next_lane.id ? "Left" : "Right");
							cout << "FSM: Lane Change " << change_state << " -> in " << (sp - car_s) << "m" 
								<< " [ s: " << sp << ", d: " << dp << " ]" << endl;
							
							lane_changes++;

							this->currentlane = next_lane.id;
							lane = next_lane.id;

							if (this->currentlane == optimal_lane.id) { break; }
							
							next_lane = nextlane(this->lanes, this->lanes[currentlane], optimal_lane);
							// update reference velocity
							temptv = this->getOptimalVelocity(this->lanes[currentlane], next_lane, max_s, dp, tempv);
						}
					}
				}

				tempv = this->getVelocity(tempv, temptv, CYCLES_PS);
			}
			
			if (lane_changes == 0) {
				cout << "FSM: (abort) Lane Change!" << endl;

				lane = this->ego.lane;
				change_lane = false;
			}
			else {
				// CK 150927
				//double s_next = car_s + this->ego.length;
				//vector<double> wp_next = this->environment.getXY(s_next, car_d);
				//solverpath.addWaypoint(wp_next[0], wp_next[1]);
			}
		}
	}
	
	if (cold_start || !change_lane) {
		
		cout << "FSM: Keep Lane" << endl;

		//In Frenet add evenly spaced ahead of the starting reference
		for (double dist = PLAN_DIST; dist < MAX_FSENSE_DIST; dist += PLAN_DIST) {
			double s = car_s + dist;
			vector<double> wp = this->environment.getXY(s, this->environment.laneCenter(lane, s));
			solverpath.addWaypoint(wp[0], wp[1]);

			max_s = max(max_s, s);
		}
	}

	// TRAJECTORY //
	cout << "Optimal velocity: " << targetv << ", current: " << ref_vel << endl;
	bool abortprev = !this->_replan && (m_offset + this->mean_off >= this->path.length());

	if (this->_replan || abortprev) {
		// DEBUG
		/*
		for (unsigned int i = 0; i < solverpath.length(); i++) {
			double x = solverpath.points[i].x;
			double y = solverpath.points[i].y;
			vector<double> frenet = this->environment.getFrenet(x, y, 0);
			cout << " Solver -> WP(" << i << ") [ x: " << x << ", y: " << y 
				<< "  |  s: " << frenet[0] << ", d: " << frenet[1] << " ]" << endl;
		}
		*/
		vector<double> wp_smooth = this->environment.getXY(max_s + PLAN_DIST, this->environment.laneCenter(lane, max_s + PLAN_DIST));
		solverpath.addWaypoint(wp_smooth[0], wp_smooth[1]);

		// localise points to car reference angle
		cout << "Local reference -> x: " << ref_x << ", y: " << ref_y << ", theta: " << ref_yaw << endl;
		solverpath.localisePoints(ref_x, ref_y, ref_yaw);

		// DEBUG
		/*
		for (unsigned int i = 0; i < solverpath.length(); i++) {
			double x = solverpath.points[i].x;
			double y = solverpath.points[i].y;
			cout << " Solver -> WP(" << i << ") [ x: " << x << ", y: " << y << " ]" << endl;
		}
		*/
		this->mean_off = (this->mean_off + m_offset) / 2.0;
		if (cold_start) { this->iter++; }
		if (change_lane) { this->_replan = false; }
		if (abortprev) { this->_replan = true; }

		double tplan = (change_lane && !this->_replan ? MAX_LANE_T : MAX_T);

		return this->trajectory(solverpath, tplan, m_offset, ref_vel, targetv, StateBase(ref_x, ref_y, ref_yaw));
	}
	else {
		if (abs_distance(car_d, this->lanes[this->currentlane].center) <= 1.0) {
			this->_replan = true;
		}

		return this->path;
	}
}

Path Model::trajectory(Path solverpath, double max_t, int m_offset, double ref_vel, double targetv, StateBase ref_state) {
	
	double ref_x = ref_state.x;
	double ref_y = ref_state.y;
	double ref_yaw = ref_state.theta;

	Path solution;
	// check path validity
	if (solverpath.valid()) {
	// create a spline
		spline s;
		// set (x,y) points to spline
		s.set_points(solverpath.xPoints(), solverpath.yPoints());
				
		double x_zero = 0;
		// add previous points
		if (this->path.length() > 0 && m_offset > 0) {
			for (int i = max(0, m_offset - MIN_PATH_LENGTH); i < m_offset; i++)
			{
				solution.addWaypoint(this->path.points[i].x, this->path.points[i].y);
			}

			x_zero = translateLocalX(solution.points[solution.length()-1].x, 
									solution.points[solution.length()-1].y, ref_yaw, ref_x, ref_y);
		}
		
		this->velocities.clear();

		for(int i = 0; i < (max_t / CYCLES_PS); i++)
		{
			double x_point = x_zero + (ref_vel * CYCLES_PS);
			double y_point = s(x_point);

			x_zero = x_point;
			
			double xlocal = x_point;
			double ylocal = y_point;
			
			// rotate back to normal after rotating it earlier
			x_point = translateGlobalX(xlocal, ylocal, ref_yaw, ref_x);
			y_point = translateGlobalY(xlocal, ylocal, ref_yaw, ref_y);
			// add solution waypoint
			solution.addWaypoint(x_point, y_point);
			// add velocity reference
			this->velocities.push_back(ref_vel);
			// update velocity with acceleration
			ref_vel = this->getVelocity(ref_vel, targetv, CYCLES_PS);
		}

		this->currentvel = ref_vel;
		this->path = solution;
	}
	else {
		solution = this->path.copy();
		this->path.clear();
	}

	cout << " Solution... " << solution.length() << endl;
	// DEBUG
	for (unsigned int i = 0; i < 8; i++) {
		double x = solution.points[i].x;
		double y = solution.points[i].y;
		vector<double> frenet = this->environment.getFrenet(x, y, ref_yaw);
		cout << " -> WP(" << i << ") [ x: " << x << ", y: " << y 
			 << "  |  s: " << frenet[0] << ", d: " << frenet[1] << " ]" << endl;
	}
	cout << "---" << endl;
	for (unsigned int i = (solution.length() - 8); i < solution.length(); i++) {
		double x = solution.points[i].x;
		double y = solution.points[i].y;
		vector<double> frenet = this->environment.getFrenet(x, y, ref_yaw);
		cout << " -> WP(" << i << ") [ x: " << x << ", y: " << y 
			 << "  |  s: " << frenet[0] << ", d: " << frenet[1] << " ]" << endl;
	}

	return solution;
}