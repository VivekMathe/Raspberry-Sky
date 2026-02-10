#pragma once

#include <Eigen/Dense>
#include <Navio2/RCInput_Navio2.h> 

// Renamed class to avoid collision
// Inherit from RCInput_Navio2
class RCInputHandler : public RCInput_Navio2
{
public:
    RCInputHandler();

    // Your custom matrix function
    Eigen::Matrix<double, 6, 1> read_ppm_vector();

    // We do NOT need to redeclare initialize() or read() or open_channel()
};