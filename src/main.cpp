#include <Eigen/Dense>
#include <iostream>

int main() {
    // Create a 2x2 matrix
    Eigen::Matrix2d A;
    A << 1, 3,
        3, 4;

    // Print matrix
    std::cout << "Matrix A:\n" << A << "\n\n";

    // Compute inverse
    Eigen::Matrix2d A_inv = A.inverse();
    std::cout << "Inverse of A:\n" << A_inv << "\n\n";

    // Check multiplication
    std::cout << "A * A_inv:\n" << A * A_inv << std::endl;    
    std::cout << Navio2::addOne(3) << std::endl;
    return 0;
}