#include "guidance.h"
#include <Eigen/Dense>
#include "math_utils.h"
#include <cmath>
#include <iostream>


Guidance::Guidance(Eigen::Vector3d pos0, double x_box_min, double x_box_max, double y_box_min, double y_box_max, double numpasses, double cruise_speed, double takeoff)
{
	phase = 0;

}

Eigen::Matrix<double, 10, 1> Guidance::getTarget(Eigen::Matrix<double, 12, 1>)
{
	//returns psides, omegades, pdes, vdes
	Eigen::Matrix<double, 10, 1> commands;
	commands << 0, 0, 0, 0, 5, 5, -.5, 0, 0, 0;
	return commands;
}
