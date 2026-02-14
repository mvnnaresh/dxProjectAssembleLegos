#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "dxRobotSimulator.h"

class demo
{
public:
    explicit demo(const std::string& modelPath, bool createViewer = true);

    bool init();
    void reset();
    void update();
    void step(int steps = 1);
    void run(int stepsPerFrame = 10);
    void moveRobotToJointPos(const std::vector<double>& jointsRad);

    dxRobotViewerFactory* viewer() const { return mSim ? mSim->viewer() : nullptr; }

private:
    std::shared_ptr<dxRobotSimulator> mSim;
    mutable std::mutex mTargetMutex;
    std::array<double, 6> mTargetJoints = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    bool mHasTarget = false;
};

