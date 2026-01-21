#pragma once
#include <Eigen/Dense>

class Guidance
{
private:
	int phase; //1 = takeoff. 2 = search. 3 = descend. 4 = Hover, drop payload. 5 = retreat to some point. 6 = land. 
	int passes;
	std::pair<double, double> xbounds; 
	std::pair<double, double> ybounds; 
	double cruise;
	double takeoff_height;
	bool found;

	Eigen::Matrix<double, 10, 1> Guidance::takeoff(Eigen::Vector3d pos, Eigen::Vector3d vel);
	Eigen::Matrix<double, 10, 1> Guidance::land(Eigen::Vector3d pos, Eigen::Vector3d vel);
	Eigen::Matrix<double, 10, 1> Guidance::search(Eigen::Vector3d pos, Eigen::Vector3d vel);
	Eigen::Matrix<double, 10, 1> Guidance::hover(Eigen::Vector3d pos, Eigen::Vector3d vel);


public:
	Guidance::Guidance(Eigen::Vector3d pos0, double x_box_min, double x_box_max, double y_box_min, double y_box_max, double numpasses, double cruise_speed, double takeoff);
	Eigen::Matrix<double, 10, 1> Guidance::getTarget(Eigen::Matrix<double, 12, 1>);
};

