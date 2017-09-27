#ifndef MAP_H_
#define MAP_H_

#include <vector>
#include <iostream>

using namespace std;

class WorldMap
{

public:
	vector<double> waypoints_x;
	vector<double> waypoints_y;
	vector<double> waypoints_s;
	vector<double> waypoints_dx;
	vector<double> waypoints_dy;

	WorldMap();
	virtual ~WorldMap();

	void init(string mapFile);

	/* Returns the closest waypoint, in all directions, to the current state */
	int closestWaypoint(double x, double y) const;

	/* Returns the forward most closest waypoint */
	int nextWaypoint(double x, double y, double theta) const;

	/* Projects the current state into Frenet coordinates */
	vector<double> getFrenet(double x, double y, double theta) const;

	/* Projects the given Frenet coordinates into map space */
	vector<double> getXY(double s, double d) const;

	int lanes(int s) const;
	double laneWidth(int lane, double s) const;
	double laneOffset(int lane, double s) const;
	double laneCenter(int lane, double s) const;
	int whichLane(double s, double d) const;
};

#endif
