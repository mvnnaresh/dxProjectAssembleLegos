#include "demo.h"

#include <iomanip>
#include <iostream>
#include <unordered_set>

#include "dxKinMuJoCo.h"
#include "dxPlannerSimple.h"

demo::demo(dxMuJoCoRobotSimulator* simulator, QObject* parent)
    : QObject(parent), mSim(simulator)
{
    mTrajectoryTimer = new QTimer(this);
    mTrajectoryTimer->setInterval(20);
    connect(mTrajectoryTimer, &QTimer::timeout, this, [this]()
    {
        if (mTrajectory.empty())
        {
            mTrajectoryTimer->stop();
            return;
        }
        if (mTrajectoryIndex >= mTrajectory.size())
        {
            if (mEndBehavior == EndBehavior::StopCommands)
            {
                mTrajectoryTimer->stop();
                return;
            }

            const std::vector<double>& joints = mTrajectory.back();
            emit jointPositionsReady(joints);
            emit ctrlTargetsFromJointsReady(joints);
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

    mKin = std::make_unique<dxKinMuJoCo>(mSim->model(), mSim->data());
    buildDofGroups();
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

void demo::applyGripperClose()
{
    emit closeGripperRequested();
}

void demo::buildDofGroups()
{
    mArmDofIndices.clear();
    mGripperDofIndices.clear();

    if (!mSim || !mSim->model())
    {
        return;
    }

    mjModel* model = mSim->model();
    std::vector<bool> jointActuated(static_cast<size_t>(model->njnt), false);
    for (int aid = 0; aid < model->nu; ++aid)
    {
        if (model->actuator_trntype[aid] != mjTRN_JOINT)
        {
            continue;
        }
        const int jid = model->actuator_trnid[2 * aid];
        if (jid >= 0 && jid < model->njnt)
        {
            jointActuated[static_cast<size_t>(jid)] = true;
        }
    }

    int dofIndex = 0;
    for (int jid = 0; jid < model->njnt; ++jid)
    {
        const int type = model->jnt_type[jid];
        if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
        {
            continue;
        }

        if (jointActuated[static_cast<size_t>(jid)])
        {
            mArmDofIndices.push_back(dofIndex);
        }
        else
        {
            mGripperDofIndices.push_back(dofIndex);
        }
        ++dofIndex;
    }
}

std::vector<double> demo::extractArmDof(const std::vector<double>& dofQpos) const
{
    std::vector<double> out;
    out.reserve(mArmDofIndices.size());
    for (int idx : mArmDofIndices)
    {
        if (idx >= 0 && static_cast<size_t>(idx) < dofQpos.size())
        {
            out.push_back(dofQpos[static_cast<size_t>(idx)]);
        }
    }
    return out;
}

std::vector<double> demo::expandArmDof(const std::vector<double>& baseDof,
                                       const std::vector<double>& armDof) const
{
    std::vector<double> out = baseDof;
    const size_t limit = std::min(mArmDofIndices.size(), armDof.size());
    for (size_t i = 0; i < limit; ++i)
    {
        const int idx = mArmDofIndices[i];
        if (idx >= 0 && static_cast<size_t>(idx) < out.size())
        {
            out[static_cast<size_t>(idx)] = armDof[i];
        }
    }
    return out;
}

// Validate FK/IK by computing the current pose and attempting to solve IK back to it.
void demo::testKinematics()
{
    if (!mSim || !mSim->model())
    {
        std::cout << "[testKinematics] simulator not initialized." << std::endl;
        return;
    }

    dxKinMuJoCo kin(mSim->model(), mSim->data());
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

    dxKinMuJoCo kin(mSim->model(), mSim->data());
    dxPlannerSimple planner(&kin);
    if (!planner.init())
    {
        std::cout << "[testPlannerSimple] planner init failed." << std::endl;
        return;
    }

    const dxMuJoCoRobotState state = mSim->getRobotState();
    std::vector<double> startDof = kin.getDofQpos(state.qpos);
    if (startDof.empty())
    {
        std::cout << "[testPlannerSimple] empty joint state." << std::endl;
        return;
    }

    std::vector<double> start = startDof;
    if (start.empty())
    {
        std::cout << "[testPlannerSimple] empty arm joint state." << std::endl;
        return;
    }

    planner.setStart(start);

    const double delta = 25.0 * 3.141592653589793 / 180.0;
    std::vector<double> goal = start;
    std::vector<int> armIndices = mArmDofIndices;
    if (armIndices.empty())
    {
        const int limit = std::min(6, static_cast<int>(goal.size()));
        armIndices.reserve(static_cast<size_t>(limit));
        for (int i = 0; i < limit; ++i)
        {
            armIndices.push_back(i);
        }
    }
    for (int idx : armIndices)
    {
        if (idx >= 0 && static_cast<size_t>(idx) < goal.size())
        {
            goal[static_cast<size_t>(idx)] += delta;
        }
    }
    planner.setGoal(goal);

    dxPlannerSimple::Params params;
    params.steps = 200;
    params.debugPaths = true;
    params.checkCollisions = true;
    params.collisionDist = 0.0;
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

    const std::vector<std::vector<double>> trajectory = planner.buildTrajectory(path);
    if (trajectory.empty())
    {
        std::cout << "[testPlannerSimple] empty trajectory." << std::endl;
        return;
    }

    mTrajectory = trajectory;
    if (mTrajectory.empty())
    {
        std::cout << "[testPlannerSimple] empty trajectory." << std::endl;
        return;
    }

    emit drawTrajectory(planner.getTrajAs3DPoints(mTrajectory));

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "[testPlannerSimple] start (deg): ";
    bool first = true;
    for (int idx : armIndices)
    {
        if (idx < 0 || static_cast<size_t>(idx) >= start.size())
        {
            continue;
        }
        if (!first)
        {
            std::cout << ", ";
        }
        std::cout << (start[static_cast<size_t>(idx)] * 180.0 / 3.141592653589793);
        first = false;
    }
    std::cout << std::endl;
    std::cout << "[testPlannerSimple] goal  (deg): ";
    first = true;
    for (int idx : armIndices)
    {
        if (idx < 0 || static_cast<size_t>(idx) >= goal.size())
        {
            continue;
        }
        if (!first)
        {
            std::cout << ", ";
        }
        std::cout << (goal[static_cast<size_t>(idx)] * 180.0 / 3.141592653589793);
        first = false;
    }
    std::cout << std::endl;

    startTrajectoryPlayback();
}

// Cartesian planner test: move along +X while keeping orientation.
void demo::testPlannerCartesian()
{
    if (!mSim || !mSim->model())
    {
        std::cout << "[testPlannerCartesian] simulator not initialized." << std::endl;
        return;
    }

    dxKinMuJoCo kin(mSim->model(), mSim->data());
    dxPlannerSimple planner(&kin);
    if (!planner.init())
    {
        std::cout << "[testPlannerCartesian] planner init failed." << std::endl;
        return;
    }
    dxPlannerSimple::Params params;
    params.steps = 80;
    params.debugPaths = true;
    params.checkCollisions = true;
    params.collisionDist = 0.0;
    planner.setParams(params);

    const dxMuJoCoRobotState state = mSim->getRobotState();
    dxKinMuJoCo::PoseResult fkResult;
    if (!kin.getFK(state.qpos, fkResult))
    {
        std::cout << "[testPlannerCartesian] FK current failed." << std::endl;
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

    std::vector<double> startPose =
    {
        pos[0], pos[1], pos[2],
        quat[0], quat[1], quat[2], quat[3]
    };
    std::vector<double> goalPose = startPose;
    goalPose[0] += 0.10;

    const std::vector<double> seed = kin.getDofQpos(state.qpos);
    if (seed.empty())
    {
        std::cout << "[testPlannerCartesian] empty seed." << std::endl;
        return;
    }

    std::vector<std::vector<double>> trajectory;
    const bool ok = planner.planCartesian(startPose, goalPose, seed, trajectory, 80);
    if (!ok || trajectory.empty())
    {
        std::cout << "[testPlannerCartesian] cartesian planning failed." << std::endl;
        return;
    }

    mTrajectory.clear();
    mTrajectory.reserve(trajectory.size());
    for (const auto& point : trajectory)
    {
        std::vector<double> full = point;
        for (size_t i = 0; i < mGripperDofIndices.size(); ++i)
        {
            const int idx = mGripperDofIndices[i];
            if (idx >= 0 && static_cast<size_t>(idx) < full.size() && static_cast<size_t>(idx) < seed.size())
            {
                full[static_cast<size_t>(idx)] = seed[static_cast<size_t>(idx)];
            }
        }
        mTrajectory.push_back(std::move(full));
    }
    emit drawTrajectory(planner.getTrajAs3DPoints(mTrajectory));
    startTrajectoryPlayback();
}

void demo::setEndBehavior(EndBehavior behavior)
{
    mEndBehavior = behavior;
}
