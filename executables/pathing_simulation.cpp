#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include "controller.h"
#include "guidance.h"
#include "math_utils.h"
#include <cmath>
#include <random>
#include <iostream>
#include <fstream>
#include <algorithm>

int main() {

	Eigen::Matrix<double, 15, 1> x_true = Eigen::Matrix<double, 15, 1>::Zero();
	Eigen::Vector3d g;
	double m = .65;
	Eigen::Vector3d inertias;
	inertias << .075, .035, .1;
	g << 0, 0, 9.81;
	double freq = 100;
	double deltat = 1 / freq;

	Eigen::Vector3d Kp_outer;
	Kp_outer << 0.5, 0.5, 5;
	Eigen::Vector3d Kd_outer;
	Kd_outer << 4, 4, 4;
	Eigen::Vector3d Kp_inner;
	Kp_inner << 3.5, 3.5, 1;
	Eigen::Vector3d Kd_inner;
	Kd_inner << 0.75, 0.75, 0.75;
	std::pair<double, double> T_sat;
	T_sat.first = .2 * m * g(2); //min
	T_sat.second = 2.0 * m * g(2); //max
	std::pair<double, double> acc_sat;
	acc_sat.first = g(2) / 4; //xy
	acc_sat.second = g(2); //z
	double max_angle = 15 * M_PI / 180;


	Controller controller(Kp_outer, Kd_outer, Kp_inner, Kd_inner, T_sat, acc_sat, max_angle, m);
	Eigen::Matrix<double, 12, 1> x = Eigen::Matrix<double, 12, 1>::Zero();
	Guidance guidance(x.block(0, 0, 3, 1), 0, 9.14, 0, 4.57, 1.2, 1, 1.83);
	Eigen::Matrix<double, 10, 1> commands;
	double t = 0;
	std::ofstream outfile("path_test.csv");
	outfile << "t,n,e,d,vn,ve,vd,phi,theta,psi,p,q,r" << std::endl;
	for (t; t <= 80 + deltat; t += deltat)
	{
		outfile << t << ',';
		for (int k = 0; k < 12; k++)
		{
			outfile << x(k);
			if (k != 11)
			{
				outfile << ',';
			}
			else
			{
				outfile << std::endl;
			}
		}
		commands = guidance.getTarget(x.block(0, 0, 3, 1), x.block(3, 0, 3, 1));
		controller.update(x);
		Eigen::Matrix<double, 4, 1> controls = controller.achieveState(commands.block(0, 0, 3, 1), commands.block(3, 0, 3, 1), commands(6), commands.block(7, 0, 3, 1));
		Eigen::Matrix<double, 12, 1> xdot = get_dynamics(x, g, m, inertias, controls(0), controls.block(1, 0, 3, 1));
		x += xdot * deltat;

	}
	outfile.close();
	return 0;
}