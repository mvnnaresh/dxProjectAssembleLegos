#pragma once

#include <array>
#include <vector>

#include "dxKinMuJoCo.h"

class dxPlannerSimple
{
public:
    struct Params
    {
        int steps = 50;
        bool debugPaths = false;
        bool checkCollisions = false;
        double collisionDist = 0.0;
        bool showTrajectoryCurve = false;
    };

    explicit dxPlannerSimple(dxKinMuJoCo* kin = nullptr);

    bool init(dxKinMuJoCo* kin = nullptr);

    void setKinematics(dxKinMuJoCo* kin);

    void setStart(const std::vector<double>& start);
    void setGoal(const std::vector<double>& goal);
    void setParams(const Params& params);

    bool solve();
    const std::vector<std::vector<double>>& getPath() const;
    std::vector<std::vector<double>> buildTrajectory() const;
    std::vector<std::vector<double>> buildTrajectory(const std::vector<std::vector<double>>& path) const;
    std::vector<std::array<double, 3>> getPathAs3DPoints() const;
    std::vector<std::array<double, 3>> getTrajAs3DPoints(const std::vector<std::vector<double>>& trajectory) const;

    bool planCartesian(const std::vector<double>& startPose,
                       const std::vector<double>& goalPose,
                       const std::vector<double>& seed,
                       std::vector<std::vector<double>>& trajectory,
                       int steps = 50);

private:
    std::vector<std::vector<double>> planJoints(const std::vector<double>& start,
                                  const std::vector<double>& goal,
                                  int steps) const;
    std::vector<std::array<double, 3>> build3DPoints(const std::vector<std::vector<double>>& qpath) const;

    dxKinMuJoCo* mKin = nullptr;
    Params mParams;
    std::vector<double> mStart;
    std::vector<double> mGoal;
    std::vector<std::vector<double>> mPath;
    bool mHasStart = false;
    bool mHasGoal = false;
};
