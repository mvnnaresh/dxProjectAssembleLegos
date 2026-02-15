#pragma once

#include <vector>

#include "dxKinMuJoCo.h"

class dxPlannerSimple
{
public:
    explicit dxPlannerSimple(dxKinMuJoCo* kin = nullptr);

    bool init(dxKinMuJoCo* kin = nullptr);

    void setKinematics(dxKinMuJoCo* kin);

    std::vector<std::vector<double>> planJoints(const std::vector<double>& start,
                                  const std::vector<double>& goal,
                                  int steps = 50) const;

    bool planCartesian(const std::vector<double>& startPose,
                       const std::vector<double>& goalPose,
                       const std::vector<double>& seed,
                       std::vector<std::vector<double>>& trajectory,
                       int steps = 50);

private:
    dxKinMuJoCo* mKin = nullptr;
};
