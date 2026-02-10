#include "RCInput.h"

RCInputHandler::RCInputHandler()
{
    // The base class RCInput_Navio2 constructor handles initialization automatically
    this->initialize();
}

Eigen::Matrix<double, 6, 1> RCInputHandler::read_ppm_vector()
{
    Eigen::Matrix<double, 6, 1> pwm_matrix;

    // Channel Mapping:
    // 0: v_east, 1: v_north, 2: Throttle, 3: Rudder, 4-5: Aux
    for (int i = 0; i < 6; ++i) {
        // Call the read() method inherited from the base class
        int val = this->read(i);
        pwm_matrix(i) = val;
    }

    return pwm_matrix;
}