#include <Eigen/Dense>
#include <cmath>
Eigen::Matrix<double, 3, 3> dcmI_B(double phi, double theta, double psi)
{
    Eigen::Matrix<double, 3, 3> Z_rot;
	Eigen::Matrix<double, 3, 3> Y_rot;
	Eigen::Matrix<double, 3, 3> X_rot;

	Z_rot << std::cos(psi), -std::sin(psi), 0,
		std::sin(psi), std::cos(psi), 0,
		0, 0, 1;
	Y_rot << std::cos(theta), 0, std::sin(theta),
		0, 1, 0,
		-std::sin(theta), 0, std::cos(theta);
	X_rot << 1, 0, 0,
		0, std::cos(phi), -std::sin(phi),
		0, std::sin(phi), std::cos(phi);

	return Z_rot * Y_rot * X_rot;
}