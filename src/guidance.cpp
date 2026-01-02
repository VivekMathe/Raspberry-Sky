#include "guidance.h"
#include <Eigen/Dense>
#include "math_utils.h"
#include <cmath>
#include <iostream>


Guidance::Guidance(Eigen::Vector3d pos0, double x_box_min, double x_box_max, double y_box_min, double y_box_max, double numpasses, double cruise_speed, double takeoff)
{
	phase = 0;

}
