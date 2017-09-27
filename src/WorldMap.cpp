#include "WorldMap.h"

#include <cmath>
#include <iostream>
#include <fstream>

#include "Helpers.h"

using namespace std;

WorldMap::WorldMap() { }

WorldMap::~WorldMap() { }

void WorldMap::init(string mapFile) {
	
	ifstream in_map_(mapFile.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line)) {

		istringstream iss(line);

		double x; double y;
		float s;
		float d_x; float d_y;

		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;

		this->waypoints_x.push_back(x);
		this->waypoints_y.push_back(y);
		this->waypoints_s.push_back(s);
		this->waypoints_dx.push_back(d_x);
		this->waypoints_dy.push_back(d_y);
	}
}

int WorldMap::closestWaypoint(double x, double y) const
{

	double closestLen = distance(x, y, this->waypoints_x[0], this->waypoints_y[0]);
	int closestWaypoint = 0;

	for (int i = 1; i < this->waypoints_x.size(); i++)
	{
		double map_x = this->waypoints_x[i];
		double map_y = this->waypoints_y[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int WorldMap::nextWaypoint(double x, double y, double theta) const
{

	int closestWaypoint = this->closestWaypoint(x, y);

	double map_x = this->waypoints_x[closestWaypoint];
	double map_y = this->waypoints_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = abs(theta - heading);

	if (angle > PI / 4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> WorldMap::getFrenet(double x, double y, double theta) const
{
	int next_wp = nextWaypoint(x, y, theta);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0)
	{
		prev_wp = this->waypoints_x.size() - 1;
	}

	double n_x = this->waypoints_x[next_wp] - this->waypoints_x[prev_wp];
	double n_y = this->waypoints_y[next_wp] - this->waypoints_y[prev_wp];
	double x_x = x - this->waypoints_x[prev_wp];
	double x_y = y - this->waypoints_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - this->waypoints_x[prev_wp];
	double center_y = 2000 - this->waypoints_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(this->waypoints_x[i], this->waypoints_y[i], this->waypoints_x[i + 1], this->waypoints_y[i + 1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return { frenet_s,frenet_d };

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> WorldMap::getXY(double s, double d) const
{
	int prev_wp = -1;

	while (s > this->waypoints_s[prev_wp + 1] && (prev_wp < (int)(this->waypoints_s.size() - 1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % this->waypoints_x.size();

	double heading = atan2((this->waypoints_y[wp2] - this->waypoints_y[prev_wp]), 
												 (this->waypoints_x[wp2] - this->waypoints_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - this->waypoints_s[prev_wp]);

	double seg_x = this->waypoints_x[prev_wp] + seg_s*cos(heading);
	double seg_y = this->waypoints_y[prev_wp] + seg_s*sin(heading);

	double perp_heading = heading - PI / 2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return { x,y };
}

/* Returns the number of lanes at the given s coordinate */
int WorldMap::lanes(int s) const {
	return 3; // ignore s
}

/* Returns the width of the specified lane at the given s coordinate */
double WorldMap::laneWidth(int lane, double s) const {
	return 4.0;
}

/* Returns the offset width of the lane at the given s coordinate */
double WorldMap::laneOffset(int lane, double s) const {
	double result = 0;

	for (unsigned int i = 0; i < lane; i++) {
		result += this->laneWidth(i, s);
	}

	return result;
}

/* Returns the specified lane center d value at the given s coordinate */
double WorldMap::laneCenter(int lane, double s) const {
	return this->laneOffset(lane, s) + (this->laneWidth(lane, s) * 0.5);
}

/* Returns the id of the lane the vehicle is in. */
int WorldMap::whichLane(double s, double d) const {
	int lanes = this->lanes(s);

	double laneAccWidth = 0;
	for (unsigned int lane = 0; lane < lanes; lane++) {
		double laneWidth = this->laneWidth(lane, s);

		if ((d > laneAccWidth - (laneWidth / 4.0)) &&
			(d < laneAccWidth + laneWidth + (laneWidth / 4.0))) {
			return lane;
		}

		laneAccWidth += laneWidth;
	}
}