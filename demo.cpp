#include "demo.h"

#include <iomanip>
#include <iostream>
#include <thread>

#include "dxKinMuJoCo.h"
#include "dxPlannerSimple.h"

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
    if (mSim)
        mSim->reset();
}

void demo::update()
{
    if (!mSim)
        return;

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

    if (!mHasTarget)
        return;

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
    if (mSim)
        mSim->step(steps);
}

void demo::run(int stepsPerFrame)
{
    if (!mSim || !mSim->viewer())
        return;

    if (stepsPerFrame < 1)
        stepsPerFrame = 1;

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

void demo::testKinematics()
{
    if (!mSim || !mSim->model() || !mSim->data())
    {
        std::cout << "[testKinematics] simulator not initialized." << std::endl;
        return;
    }

    dxKinMuJoCo kin(mSim->model(), mSim->data());

    dxKinMuJoCo::PoseResult fkResult;
    const std::vector<double> qpos = mSim->getQpos();
    if (!kin.getFK(qpos, fkResult))
    {
        std::cout << "[testKinematics] FK failed." << std::endl;
        return;
    }

    const Eigen::Matrix4f& pose = std::get<1>(fkResult);
    const std::array<double, 3> pos =
    {
        static_cast<double>(pose(0, 3)),
        static_cast<double>(pose(1, 3)),
        static_cast<double>(pose(2, 3))
    };

    double mat[9] =
    {
        static_cast<double>(pose(0, 0)), static_cast<double>(pose(0, 1)), static_cast<double>(pose(0, 2)),
        static_cast<double>(pose(1, 0)), static_cast<double>(pose(1, 1)), static_cast<double>(pose(1, 2)),
        static_cast<double>(pose(2, 0)), static_cast<double>(pose(2, 1)), static_cast<double>(pose(2, 2))
    };
    double quat[4] = { 1.0, 0.0, 0.0, 0.0 };
    mju_mat2Quat(quat, mat);

    std::vector<double> target =
    {
        pos[0], pos[1], pos[2],
        quat[0], quat[1], quat[2], quat[3]
    };

    std::vector<double> solution;
    const bool ikOk = kin.getIK(qpos, target, solution);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "[testKinematics] FK pos: "
              << pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;
    std::cout << "[testKinematics] IK " << (ikOk ? "converged" : "not converged")
              << ", solution size: " << solution.size() << std::endl;
}

void demo::testPlanner()
{
    std::thread([this]()
    {
        if (!mSim || !mSim->model() || !mSim->data())
        {
            std::cout << "[testPlanner] simulator not initialized." << std::endl;
            return;
        }

        dxKinMuJoCo kin(mSim->model(), mSim->data());
        dxPlannerSimple planner(&kin);
        if (!planner.init())
        {
            std::cout << "[testPlanner] planner init failed." << std::endl;
            return;
        }

        dxKinMuJoCo::PoseResult fkResult;
        if (!kin.getFKCurrent(fkResult))
        {
            std::cout << "[testPlanner] FK current failed." << std::endl;
            return;
        }

        std::vector<double> start = std::get<2>(fkResult);
        if (start.empty())
        {
            std::cout << "[testPlanner] empty joint state." << std::endl;
            return;
        }

        const double delta = 25.0 * 3.141592653589793 / 180.0;
        std::vector<double> goal = start;
        for (double& v : goal)
            v += delta;

        const int steps = 200;
        std::vector<std::vector<double>> trajectory = planner.planJoints(start, goal, steps);
        if (trajectory.empty())
        {
            std::cout << "[testPlanner] joint planning failed." << std::endl;
            return;
        }

        const auto latency = std::chrono::milliseconds(5);
        for (const auto& point : trajectory)
        {
            if (mSim)
                mSim->setCtrlTargets(point);
            std::this_thread::sleep_for(latency);
        }
    }).detach();
}
