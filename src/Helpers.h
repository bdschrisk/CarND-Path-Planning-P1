#ifndef HELPERS_H_
#define HELPERS_H_

#include <cmath>
#include <algorithm>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

static const double PI = 3.14159265358979323846;

static double deg2rad(double x) { return x * PI / 180; }

static double rad2deg(double x) { return x * 180 / PI; }

static double clip(double v, double low, double high) { return max(low, min(v, high)); }

static double abs_distance(double x1, double x2) { return abs(x2 - x1); }

static double distance(double x, double y) { return sqrt(pow(x, 2.0) + pow(y, 2.0)); }

static double distance(double x1, double y1, double x2, double y2) {
	return sqrt(pow((x2 - x1), 2.0) + pow((y2 - y1), 2.0));
}

static double translateLocalX(const double &xglobal, const double &yglobal, const double &theta, const double &xref, const double &yref) {
	return ((xglobal - xref) * cos(0.0 - theta) - (yglobal - yref) * sin(0.0 - theta));
}

static double translateLocalY(const double &xglobal, const double &yglobal, const double &theta, const double &xref, const double &yref) {
	return ((xglobal - xref) * sin(0.0 - theta) + (yglobal - yref) * cos(0.0 - theta));
}

static double translateGlobalX(const double &xlocal, const double &ylocal, const double &theta, const double &xref) {
	return (xlocal * cos(theta) - ylocal * sin(theta)) + xref;
}

static double translateGlobalY(const double &xlocal, const double &ylocal, const double &theta, const double &yref) {
	return (xlocal * sin(theta) + ylocal * cos(theta)) + yref;
}

static double velocity(double vx, double vy) {
	return sqrt(pow(vx, 2.0) + pow(vy, 2.0));
}

/**
* Returns the final velocity.
*/
static double velocity(double vi, double a, double t) { return (vi + a * t); }
/**
* Returns the time required to reach the displacement.
*/
static double timeToDisplacement(double vi, double vf, double d) { return d / ((vi + vf) * 0.5); }
/**
* Returns the acceleration required for the given time delta.
*/
static double accelTime(double vi, double vf, double t) { return (vf - vi) / t; }
/**
* Returns the acceleration required for the given velocities and displacement.
*/
static double accelDisplacement(double vi, double vf, double d) { return (pow(vf, 2) - pow(vi, 2)) / (d * 2); }
/**
* Returns the time to reach the final velocity.
*/
static double velocityTime(double vi, double vf, double a) { return (vf - vi) / a; }
/**
* Returns the total displacement given the velocities and time delta.
*/
static double displacement(double vi, double vf, double t) { return (t * (vi + vf)) * 0.5; }

static double metric_speed(double mph) { return ((mph * 1.609344) / 3.6); }

static bool collides(double x1, double y1, double width1, double length1, 
	double x2, double y2, double width2, double length2) {
	
	return (x1 < x2 + width2 && x1 + width1 > x2 
		 && y1 < y2 + length2 && length1 + y1 > y2);
};

static double rescale(double v, double min, double max, double new_min, double new_max) {
	return clip((v - min) * (new_max - new_min) / (max - min) + new_min, new_min, new_max);
}

// Evaluate a polynomial.
static double polyEval(Eigen::VectorXd coeffs, double x) {
	double result = 0.0;

	for (int i = 0; i < coeffs.size(); i++) {
		result += coeffs[i] * pow(x, i);
	}

	return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
static Eigen::VectorXd polyFit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
	Eigen::MatrixXd A(xvals.size(), order + 1);

	for (int i = 0; i < xvals.size(); i++) {
		A(i, 0) = 1.0;
	}

	for (int j = 0; j < xvals.size(); j++) {
		for (int i = 0; i < order; i++) {
			A(j, i + 1) = A(j, i) * xvals(j);
		}
	}

	auto Q = A.householderQr();
	auto result = Q.solve(yvals);

	return result;
}

#endif