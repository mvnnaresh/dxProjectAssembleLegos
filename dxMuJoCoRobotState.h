#pragma once

#include <array>
#include <vector>

struct dxMuJoCoRobotState
{
    std::vector<double> jointConf;
    std::vector<double> jointVel;
    std::vector<double> actuatorInput;
    std::vector<double> worldQpos;
    std::array<double, 3> eePos = { 0.0, 0.0, 0.0 };
    std::array<double, 4> eeQuat = { 1.0, 0.0, 0.0, 0.0 };
};
