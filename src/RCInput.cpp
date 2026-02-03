#include "RCInput.h"

void RCInput_Navio2::initialize() {
    // Left empty because logic is in the Constructor
}

RCInput_Navio2::RCInput_Navio2()
{
    // Initialisation of channels, opened channels

    for (size_t i = 0; i < CHANNEL_COUNT; i++) {
        channels[i] = open_channel(i);
    }
}

Eigen::Matrix<double, 6, 1> RCInput_Navio2::read_ppm_vector()
{
    Eigen::Matrix<double, 6, 1> pwm_matrix;

    // Channel 0: Aileron
    // Channel 1: Elevator
    // Channel 2: Throttle
    // Channel 3: Rudder
    // Channel 4: Aux Channel
    // Channel 5: Aux Channel

    for (int i = 0; i < 6; ++i) {
        int val = this->read(i);

        pwm_matrix(i) = val;
    }

    return pwm_matrix;
}