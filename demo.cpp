#include "demo.h"

#include <iostream>
#include <thread>

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
    if (!mSim) return;

    {
        std::lock_guard<std::mutex> lock(mTrajectoryMutex);
        if (mHasTrajectory && mTrajectoryIndex < mTrajectory.size())
        {
            const std::array<double, 6>& joints = mTrajectory[mTrajectoryIndex++];
            std::vector<double> targets(joints.begin(), joints.end());
            mSim->setCtrlTargets(targets);
            if (mTrajectoryIndex >= mTrajectory.size())
                mHasTrajectory = false;
            return;
        }
    }

    if (!mHasTarget) return;

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

void demo::test()
{
    std::thread([this]()
    {
        std::cout << "Press Enter to start trajectory..." << std::endl;
        std::string line;
        std::getline(std::cin, line);

        const std::array<double, 6> target = {0.35, -1.2, 1.4, -1.3, -1.57, 0.25};

        std::array<double, 6> start = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        if (mSim)
        {
            const std::vector<double> ctrl = mSim->getCtrl();
            for (int i = 0; i < 6 && i < static_cast<int>(ctrl.size()); ++i)
                start[i] = ctrl[i];
        }

        auto lerp = [](double a, double b, double t)
        {
            return a + (b - a) * t;
        };

        const int totalSteps = 200;
        const auto latency = std::chrono::milliseconds(5);
        for (int i = 0; i <= totalSteps; ++i)
        {
            const double t = static_cast<double>(i) / static_cast<double>(totalSteps);
            std::array<double, 6> joints;
            for (int j = 0; j < 6; ++j)
                joints[j] = lerp(start[j], target[j], t);

            moveRobotToJointPos(std::vector<double>(joints.begin(), joints.end()));
            std::this_thread::sleep_for(latency);
        }
    }).detach();
}
