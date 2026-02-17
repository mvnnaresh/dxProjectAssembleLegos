#include "demo.h"

#include <iomanip>
#include <iostream>

#include "dxKinMuJoCo.h"
#include "dxPlannerSimple.h"

demo::demo(dxMuJoCoRobotSimulator* simulator, QObject* parent)
    : QObject(parent), mSim(simulator)
{
    mTrajectoryTimer = new QTimer(this);
    mTrajectoryTimer->setInterval(20);
    connect(mTrajectoryTimer, &QTimer::timeout, this, [this]()
    {
        if (mTrajectoryIndex >= mTrajectory.size())
        {
            mTrajectoryTimer->stop();
            return;
        }

        const std::vector<double>& joints = mTrajectory[mTrajectoryIndex++];
        emit jointPositionsReady(joints);
        emit ctrlTargetsFromJointsReady(joints);
    });
}

// Initialize kinematics helper after the simulator has a model loaded.
bool demo::init()
{
    if (!mSim || !mSim->model())
    {
        return false;
    }

    mKin = std::make_unique<dxKinMuJoCo>(mSim->model(), nullptr);
    return true;
}

void demo::startTrajectoryPlayback()
{
    if (!mTrajectoryTimer)
    {
        return;
    }
    if (mTrajectory.empty())
    {
        return;
    }
    mTrajectoryIndex = 0;
    if (!mTrajectoryTimer->isActive())
    {
        mTrajectoryTimer->start();
    }
}

// Validate FK/IK by computing the current pose and attempting to solve IK back to it.
void demo::testKinematics()
{
    if (!mSim || !mSim->model())
    {
        std::cout << "[testKinematics] simulator not initialized." << std::endl;
        return;
    }

    dxKinMuJoCo kin(mSim->model(), nullptr);
    const dxMuJoCoRobotState state = mSim->getRobotState();

    dxKinMuJoCo::PoseResult fkResult;
    if (!kin.getFK(state.qpos, fkResult))
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
    const bool ikOk = kin.getIK(state.qpos, target, solution);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "[testKinematics] FK pos: "
              << pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;
    std::cout << "[testKinematics] IK " << (ikOk ? "converged" : "not converged")
              << ", solution size: " << solution.size() << std::endl;
}

// Simplified planner test: start from current state, add +25 deg to arm joints, then execute.
void demo::testPlannerSimple()
{
    if (!mSim || !mSim->model())
    {
        std::cout << "[testPlannerSimple] simulator not initialized." << std::endl;
        return;
    }

    dxKinMuJoCo kin(mSim->model(), nullptr);
    dxPlannerSimple planner(&kin);
    if (!planner.init())
    {
        std::cout << "[testPlannerSimple] planner init failed." << std::endl;
        return;
    }

    const dxMuJoCoRobotState state = mSim->getRobotState();
    std::vector<double> start = kin.getDofQpos(state.qpos);
    if (start.empty())
    {
        std::cout << "[testPlannerSimple] empty joint state." << std::endl;
        return;
    }

    planner.setStart(start);

    const double delta = 25.0 * 3.141592653589793 / 180.0;
    std::vector<double> goal = start;
    const int dof = static_cast<int>(start.size());
    const int limit = std::min(dof, static_cast<int>(goal.size()));
    for (int i = 0; i < 6; ++i)
    {
        goal[i] += delta;
    }
    planner.setGoal(goal);

    dxPlannerSimple::Params params;
    params.steps = 200;
    params.debugPaths = false;
    planner.setParams(params);

    if (!planner.solve())
    {
        std::cout << "[testPlannerSimple] joint planning failed." << std::endl;
        return;
    }

    const std::vector<std::vector<double>>& path = planner.getPath();
    if (path.empty())
    {
        std::cout << "[testPlannerSimple] empty planned path." << std::endl;
        return;
    }

    mTrajectory = planner.buildTrajectory(path);
    if (mTrajectory.empty())
    {
        std::cout << "[testPlannerSimple] empty trajectory." << std::endl;
        return;
    }

    emit drawTrajectory(planner.getTrajAs3DPoints(mTrajectory));

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "[testPlannerSimple] start (deg): ";
    for (size_t i = 0; i < start.size(); ++i)
    {
        if (i > 0)
        {
            std::cout << ", ";
        }
        std::cout << (start[i] * 180.0 / 3.141592653589793);
    }
    std::cout << std::endl;
    std::cout << "[testPlannerSimple] goal  (deg): ";
    for (size_t i = 0; i < goal.size(); ++i)
    {
        if (i > 0)
        {
            std::cout << ", ";
        }
        std::cout << (goal[i] * 180.0 / 3.141592653589793);
    }
    std::cout << std::endl;

    startTrajectoryPlayback();
}
