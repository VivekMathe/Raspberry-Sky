#ifndef EKF_H
#define EKF_H
#include <Eigen/Dense>
class EKF
{
private:
	Eigen::Matrix<double, 15, 1> x;
	Eigen::Matrix<double, 15, 15> P;
	Eigen::Matrix<double, 15, 15> A;
	Eigen::Matrix<double, 12, 12> Q;
	Eigen::Matrix4d R;
	Eigen::Vector3d g;
	Eigen::Vector3d alpha;
	Eigen::Vector3d body_accels;
	Eigen::Vector3d radius;
	Eigen::Vector3d omega_measured;
	double dt;

public:
	//Constructor. Need gravity vector in NED, 
	EKF::EKF(Eigen::Vector3d r, Eigen::Matrix<double, 15, 1> x0, Eigen::Vector3d gyro0, Eigen::Vector3d accel0, Eigen::Matrix<double, 12, 1> sigmaw, Eigen::Vector4d sigmav, double freq);
	void estimate(Eigen::Vector3d omega, Eigen::Vector3d body_accels);
	void update(Eigen::Vector4d m);
	Eigen::Matrix<double, 15, 1> getState();
	Eigen::Matrix<double, 12, 1> getControlState();
	Eigen::Vector3d getOmega();
};

#endif