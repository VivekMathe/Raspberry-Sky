#include <Eigen/Dense>
#include <iostream>
#include <ekflib/dcm.h>
int main() {
	Eigen::Matrix<double,3,3> x = dcmI_B(5, 5, 5);
	std::cout << x << std::endl;
}