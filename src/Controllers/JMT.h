#ifndef JMT_H_
#define JMT_H_

#include <vector>
#include <cmath>
#include "../Eigen-3.3/Eigen/Dense"

using namespace std;
using namespace Eigen;

struct JState {
	double s;
	double s_d1;
	double s_d2;

	JState(double s, double s_dot, double s_dot_dot) {
		this->s = s;
		this->s_d1 = s_dot;
		this->s_d2 = s_dot_dot;
	}
};

class JMT {

public:

	JMT() { }
	virtual ~JMT() { }

	VectorXd minimise(JState start, JState end, double T) {
		/*
		Calculate the Jerk Minimizing Trajectory that connects the initial state
		to the final state in time T.

		INPUTS

		start - the vehicles start location given as a length three array
		corresponding to initial values of [s, s_dot, s_double_dot]

		end   - the desired end state for vehicle. Like "start" this is a
		length three array.

		T     - The duration, in seconds, over which this maneuver should occur.

		OUTPUT
		an array of length 6, each value corresponding to a coefficent in the polynomial
		s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

		EXAMPLE

		> JMT( [0, 10, 0], [10, 10, 0], 1)
		[0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
		*/

		double tp2 = pow(T, 2);
		double tp3 = pow(T, 3);
		double tp4 = pow(T, 4);
		double tp5 = pow(T, 5);

		MatrixXd A = MatrixXd(3, 3);
		A <<	tp3, tp4, tp5,
					3 * tp2, 4 * tp3, 5 * tp4,
					6 * T, 12 * tp2, 20 * tp3;

		MatrixXd B = MatrixXd(3, 1);
		B <<	end.s - (start.s + start.s_d1 * T + 0.5 * start.s_d2 * tp2),
					end.s_d1 - (start.s_d1 + start.s_d2 * T),
					end.s_d2 - start.s_d2;

		MatrixXd Ai = A.inverse();

		MatrixXd C = Ai * B;

		VectorXd coeffs = VectorXd::Zero(C.size() + 3);
		coeffs(0) = start.s;
		coeffs(1) = start.s_d1;
		coeffs(2) = 0.5 * start.s_d2;

		for (int i = 0; i < C.size(); i++)
		{
			coeffs(i + 3) = C.data()[i];
		}

		return coeffs;
	}
};

#endif