#ifndef PATH_H_
#define PATH_H_

#include <vector>
#include <iostream>
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
			double temp_x = this->points[i].x;
			double temp_y = this->points[i].y;
			
			this->points[i].x = translateLocalX(temp_x, temp_y, theta, xref, yref);
			this->points[i].y = translateLocalY(temp_x, temp_y, theta, xref, yref);
		}
	};

	void globalisePoints(double xref, double yref, double theta) {
		for (unsigned int i = 0; i < this->points.size(); i++) {
			double temp_x = this->points[i].x;
			double temp_y = this->points[i].y;
			
			this->points[i].x = translateGlobalX(temp_x, temp_y, theta, xref);
			this->points[i].y = translateGlobalY(temp_x, temp_y, theta, yref);
		}
	}

	void addWaypoint(double x, double y) {
		State state(x, y, 0);
		this->points.push_back(state);
	};

	void clear() {
		this->points.clear();
	}

	Path copy() {
		Path result;
		for (unsigned int i = 0; i < this->length(); i++) {
			result.addWaypoint(this->points[i].x, this->points[i].y);
		}
		return result;
	}

	bool valid() {
		int length = this->length();

		if (length < 2) return false;
		else {
			double px = this->points[this->length() - 1].x;
			
			for (int i = length - 2; i >= 0; i--) {
				if (px <= this->points[i].x) {
					cout << "Point " << i << " | " << this->points[i].x << " --- <= --- " << px << endl;
					return false;
				}
				px = this->points[i].x;
			}
		}

		return true;
	}
};

#endif