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

static double distance(double x1, double x2) {
	return sqrt(pow((x2 - x1), 2.0));
}

static double distance(double x1, double y1, double x2, double y2) {
	return sqrt(pow((x2 - x1), 2.0) + pow((y2 - y1), 2.0));
}

static double translateLocalX(double xref, double yref, double theta, double x, double y) {
	return ((x - xref) * cos(0 - theta) - (y - yref) * sin(0 - theta));
}

static double translateLocalY(double xref, double yref, double theta, double x, double y) {
	return ((x - xref) * sin(0 - theta) + (y - yref) * cos(0 - theta));
}

static double translateGlobalX(double xlocal, double ylocal, double theta, double x, double y) {
	return (xlocal * cos(theta) - ylocal * sin(theta)) + x;
}

static double translateGlobalY(double xlocal, double ylocal, double theta, double x, double y) {
	return (xlocal * sin(theta) + ylocal * cos(theta)) + y;
}

static double velocity(double vx, double vy) {
	return sqrt(pow(vx, 2.0) + pow(vy, 2.0));
}

static double metric_speed(double mph) {
	return mph * 1.609344;
}

static bool collides(double x1, double y1, double width1, double length1, 
	double x2, double y2, double width2, double length2) {
	
	bool result = (x1 < x2 + width1 && x1 + width2 > x2);
	result &= (y1 < y2 + length1 && y2 + length2 > y2);

	return result;
};

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