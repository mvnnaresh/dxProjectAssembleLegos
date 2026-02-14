#include "demo.h"

demo::demo(const std::string& modelPath, bool createViewer)
    : mSim(std::make_shared<dxRobotSimulator>(modelPath, createViewer))
{
}

bool demo::init()
{
    return mSim ? mSim->init() : false;
}

void demo::reset()
{
    if (mSim) mSim->reset();
}

void demo::update()
{
    if (!mSim || !mHasTarget) return;

    std::array<double, 6> joints;
    {
        std::lock_guard<std::mutex> lock(mTargetMutex);
        joints = mTargetJoints;
    }

    std::vector<double> targets(joints.begin(), joints.end());
    mSim->setCtrlTargets(targets);
}

void demo::step(int steps)
{
    if (mSim) mSim->step(steps);
}

void demo::run(int stepsPerFrame)
{
    if (!mSim || !mSim->viewer()) return;
    if (stepsPerFrame < 1) stepsPerFrame = 1;

    while (mSim->viewer() && !mSim->viewer()->shouldClose())
    {
        update();
        step(stepsPerFrame);
        mSim->viewer()->renderOnce();
    }
}

void demo::moveRobotToJointPos(const std::vector<double>& jointsRad)
{
    if (jointsRad.size() < 6) return;
    std::lock_guard<std::mutex> lock(mTargetMutex);
    for (int i = 0; i < 6; ++i)
        mTargetJoints[i] = jointsRad[i];
    mHasTarget = true;
}
