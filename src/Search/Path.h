#ifndef PATH_H_
#define PATH_H_

#include <vector>
#include "State.h"
#include "../Helpers.h"

using namespace std;

struct Path {
	bool found = false;
	vector<State> points;

	int length() {
		return this->points.size();
	}

	/* Unravels the x coordinates in the path solution */
	vector<double> xPoints() {
		vector<double> result;

		for (unsigned int i = 0; i < this->points.size(); i++) {
			double d = this->points[i].x;
			result.push_back(d);
		}

		return result;
	}

	/* Unravels the y coordinates in the path solution */
	vector<double> yPoints() {
		vector<double> result;

		for (unsigned int i = 0; i < this->points.size(); i++) {
			double d = this->points[i].y;
			result.push_back(d);
		}

		return result;
	}

	void localisePoints(double xref, double yref, double theta) {
		for (unsigned int i = 0; i < this->points.size(); i++) {
			this->points[i].x = translateLocalX(xref, yref, theta, this->points[i].x, this->points[i].y);
			this->points[i].y = translateLocalY(xref, yref, theta, this->points[i].x, this->points[i].y);
		}
	};

	void addWaypoint(double x, double y) {
		State state(x, y, 0);
		this->points.push_back(state);
	};
};

#endif