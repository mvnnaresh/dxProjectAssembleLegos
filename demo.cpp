#include "demo.h"

#include <unordered_set>
#include <cstring>
#include <iostream>
#include <QEventLoop>
#include <QMetaObject>
#include <QTimer>

#include "dxKinMuJoCo.h"
#include "dxPlannerSimple.h"

namespace
{
constexpr double kGripperOpenRatio = 0.0;
constexpr double kGripperCloseRatio = 0.96;
}

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
                emit trajectoryFinished();
                return;
            }

            const std::vector<double>& joints = mTrajectory.back();
            emit ctrlTargetsFromJointsReady(joints);
            return;
        }

        if (mGripperEventIndex < mGripperEvents.size() &&
                mTrajectoryIndex == mGripperEvents[mGripperEventIndex].first)
        {
            const double value = mGripperEvents[mGripperEventIndex].second;
            emit gripperPositionRequested(value);
            mGripperLatched = true;
            mGripperLatchedValue = value;
            ++mGripperEventIndex;
        }

        if (mGripperLatched)
        {
            emit gripperPositionRequested(mGripperLatchedValue);
        }

        const std::vector<double>& joints = mTrajectory[mTrajectoryIndex++];
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

    mSim->enableHardLock(false);
    mSim->enablePdHold(false);

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
    mGripperEventIndex = 0;
    mGripperLatched = false;
    mGripperLatchedValue = 0.0;
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

        const char* jointName = mj_id2name(model, mjOBJ_JOINT, jid);
        const bool isFinger = jointName && std::strstr(jointName, "finger");
        if (isFinger)
        {
            mGripperDofIndices.push_back(dofIndex);
        }
        else if (jointActuated[static_cast<size_t>(jid)])
        {
            mArmDofIndices.push_back(dofIndex);
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
        return;
    }

    dxKinMuJoCo kin(mSim->model(), mSim->data());
    const dxMuJoCoRobotState state = mSim->getRobotState();

    dxKinMuJoCo::PoseResult fkResult;
    if (!kin.getFK(state.jointConf, fkResult))
    {
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
    const bool ikOk = kin.getIK(state.jointConf, target, solution);

    (void)ikOk;
    (void)solution;
}

// Simplified planner test: start from current state, add +25 deg to arm joints, then execute.
void demo::testPlannerSimple()
{
    if (!mSim || !mSim->model())
    {
        return;
    }

    dxKinMuJoCo kin(mSim->model(), mSim->data());
    dxPlannerSimple planner(&kin);
    if (!planner.init())
    {
        return;
    }

    const dxMuJoCoRobotState state = mSim->getRobotState();
    std::vector<double> startDof = state.jointConf;
    if (startDof.empty())
    {
        return;
    }

    std::vector<double> start = startDof;
    if (start.empty())
    {
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
    params.checkCollisions = false;
    params.collisionDist = -0.001;
    planner.setParams(params);

    if (!planner.solve())
    {
        return;
    }

    const std::vector<std::vector<double>>& path = planner.getPath();
    if (path.empty())
    {
        return;
    }

    const std::vector<std::vector<double>> trajectory = planner.buildTrajectory(path);
    if (trajectory.empty())
    {
        return;
    }

    mTrajectory = trajectory;
    if (mTrajectory.empty())
    {
        return;
    }

    {
        dxPlannerSimple vizPlanner(mKin.get());
        if (vizPlanner.init())
        {
            emit drawTrajectory(vizPlanner.getTrajAs3DPoints(mTrajectory));
        }
    }

    startTrajectoryPlayback();
}

// Cartesian planner test: move along +X while keeping orientation.
void demo::testPlannerCartesian()
{
    if (!mSim || !mSim->model())
    {
        return;
    }

    dxKinMuJoCo kin(mSim->model(), mSim->data());
    dxPlannerSimple planner(&kin);
    if (!planner.init())
    {
        return;
    }
    dxPlannerSimple::Params params;
    params.steps = 80;
    params.debugPaths = true;
    params.checkCollisions = true;
    params.collisionDist = -0.001;
    planner.setParams(params);

    const dxMuJoCoRobotState state = mSim->getRobotState();
    dxKinMuJoCo::PoseResult fkResult;
    if (!kin.getFK(state.jointConf, fkResult))
    {
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

    const std::vector<double> seed = state.jointConf;
    if (seed.empty())
    {
        return;
    }

    std::vector<std::vector<double>> trajectory;
    const bool ok = planner.planCartesian(startPose, goalPose, seed, trajectory, 80);
    if (!ok || trajectory.empty())
    {
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

void demo::testPickAndPlace()
{
    if (!mSim || !mSim->model() || !mKin)
    {
        return;
    }

    dxPlannerSimple planner(mKin.get());
    if (!planner.init())
    {
        return;
    }
    dxPlannerSimple::Params params;
    params.steps = 80;
    params.debugPaths = true;
    params.checkCollisions = true;
    params.collisionDist = 0.0;
    planner.setParams(params);

    const dxMuJoCoRobotState state = mSim->getRobotState();
    if (state.jointConf.empty())
    {
        return;
    }

    dxKinMuJoCo::PoseResult fkResult;
    if (!mKin->getFK(state.jointConf, fkResult))
    {
        return;
    }

    const Eigen::Matrix4f& pose = std::get<1>(fkResult);
    double mat[9] =
    {
        static_cast<double>(pose(0, 0)), static_cast<double>(pose(0, 1)), static_cast<double>(pose(0, 2)),
        static_cast<double>(pose(1, 0)), static_cast<double>(pose(1, 1)), static_cast<double>(pose(1, 2)),
        static_cast<double>(pose(2, 0)), static_cast<double>(pose(2, 1)), static_cast<double>(pose(2, 2))
    };
    double quat[4] = { 1.0, 0.0, 0.0, 0.0 };
    mju_mat2Quat(quat, mat);

    std::vector<double> cubePose = mSim->getBodyPoseByName("grasp_cube");
    if (cubePose.empty())
    {
        cubePose = mSim->getBodyPoseByName("cube");
    }
    if (cubePose.empty())
    {
        cubePose = mSim->getBodyPoseByName("object");
    }
    if (cubePose.size() < 3)
    {
        return;
    }

    const double cubeX = cubePose[0];
    const double cubeY = cubePose[1];
    const double cubeZ = cubePose[2];

    auto makePose = [&](double x, double y, double z)
    {
        return std::vector<double>
        {
            x, y, z,
            quat[0], quat[1], quat[2], quat[3]
        };
    };

    const double pregraspZ = cubeZ + 0.10;
    const double graspZ = cubeZ + 0.02;
    const double liftZ = cubeZ + 0.12;
    const double placeX = cubeX + 0.20;
    const double placeY = cubeY;

    std::vector<double> startPose =
    {
        static_cast<double>(pose(0, 3)),
        static_cast<double>(pose(1, 3)),
        static_cast<double>(pose(2, 3)),
        quat[0], quat[1], quat[2], quat[3]
    };

    std::vector<double> seed = state.jointConf;
    std::vector<std::vector<double>> combined;
    std::vector<std::pair<size_t, double>> gripperEvents;

    auto appendSegment = [&](const std::vector<double>& goalPose, int steps, bool checkCollisions)
    {
        params.checkCollisions = checkCollisions;
        planner.setParams(params);
        std::vector<std::vector<double>> segment;
        if (!planner.planCartesian(startPose, goalPose, seed, segment, steps))
        {
            return false;
        }
        if (segment.empty())
        {
            return false;
        }

        for (auto& point : segment)
        {
            for (int idx : mGripperDofIndices)
            {
                if (idx >= 0 && static_cast<size_t>(idx) < point.size() &&
                        static_cast<size_t>(idx) < seed.size())
                {
                    point[static_cast<size_t>(idx)] = seed[static_cast<size_t>(idx)];
                }
            }
        }

        if (!combined.empty())
        {
            segment.erase(segment.begin());
        }

        combined.insert(combined.end(), segment.begin(), segment.end());
        seed = combined.back();
        startPose = goalPose;
        return true;
    };

    auto appendHold = [&](size_t holdSteps)
    {
        if (combined.empty() || holdSteps == 0)
        {
            return;
        }
        const std::vector<double> last = combined.back();
        combined.insert(combined.end(), holdSteps, last);
    };

    const std::vector<double> pregraspPose = makePose(cubeX, cubeY, pregraspZ);
    const std::vector<double> graspPose = makePose(cubeX, cubeY, graspZ);
    const std::vector<double> liftPose = makePose(cubeX, cubeY, liftZ);
    const std::vector<double> placeAbovePose = makePose(placeX, placeY, liftZ);
    const std::vector<double> placePose = makePose(placeX, placeY, graspZ);

    if (!appendSegment(pregraspPose, 60, false))
    {
        return;
    }
    if (!appendSegment(graspPose, 40, false))
    {
        return;
    }

    if (!combined.empty())
    {
        gripperEvents.emplace_back(0, kGripperOpenRatio);
        gripperEvents.emplace_back(combined.size() - 1, kGripperCloseRatio);
    }
    appendHold(25);

    if (!appendSegment(liftPose, 40, false))
    {
        return;
    }
    if (!appendSegment(placeAbovePose, 60, false))
    {
        return;
    }
    if (!appendSegment(placePose, 40, false))
    {
        return;
    }

    if (!combined.empty())
    {
        gripperEvents.emplace_back(combined.size() - 1, kGripperOpenRatio);
    }
    appendHold(20);

    mTrajectory = std::move(combined);
    mGripperEvents = std::move(gripperEvents);
    emit drawTrajectory(planner.getTrajAs3DPoints(mTrajectory));
    startTrajectoryPlayback();
}

void demo::testPickAndPlace3()
{
    if (!mSim || !mSim->model() || !mKin)
    {
        return;
    }

    const dxMuJoCoRobotState state = mSim->getRobotState();
    if (state.jointConf.empty())
    {
        return;
    }

    std::vector<double> cubePose = mSim->getBodyPoseByName("grasp_cube");
    const double cubeX = cubePose[0];
    const double cubeY = cubePose[1];
    const double cubeZ = cubePose[2];

    const double pregraspZ = cubeZ + 0.10;
    const double graspZ = cubeZ + 0.02;
    const double liftZ = cubeZ + 0.12;
    const double placeX = cubeX + 0.20;
    const double placeY = cubeY;
    const double placeZ = cubeZ + 0.04;

    auto makePose = [&](double x, double y, double z)
    {
        return std::vector<double>
        {
            x, y, z,
            state.eeQuat[0], state.eeQuat[1], state.eeQuat[2], state.eeQuat[3]
        };
    };

    const std::vector<double> startPose =
    {
        state.eePos[0], state.eePos[1], state.eePos[2],
        state.eeQuat[0], state.eeQuat[1], state.eeQuat[2], state.eeQuat[3]
    };
    const std::vector<double> pregraspPose = makePose(cubeX, cubeY, pregraspZ);
    const std::vector<double> graspPose = makePose(cubeX, cubeY, graspZ);
    const std::vector<double> preplacePose = makePose(placeX, placeY, liftZ);
    const std::vector<double> placePose = makePose(placeX, placeY, placeZ);

    openGripper();
    waitSteps(10);

    sendRobotTo(startPose, pregraspPose, 60);
    std::cout << "[testPickAndPlace3] reached pregrasp" << std::endl;

    sendRobotTo(pregraspPose, graspPose, 40);
    std::cout << "[testPickAndPlace3] reached grasp" << std::endl;

    closeGripper();
    waitSteps(25);

    sendRobotTo(graspPose, pregraspPose, 40);
    std::cout << "[testPickAndPlace3] returned to pregrasp" << std::endl;

    sendRobotTo(pregraspPose, preplacePose, 60);
    std::cout << "[testPickAndPlace3] reached preplace" << std::endl;

    sendRobotTo(preplacePose, placePose, 40);
    std::cout << "[testPickAndPlace3] reached place" << std::endl;

    openGripper();
    waitSteps(40);

    sendRobotTo(placePose, preplacePose, 40);
    std::cout << "[testPickAndPlace3] returned to preplace" << std::endl;
}

bool demo::sendRobotTo(const std::vector<double>& fromPose,
                       const std::vector<double>& toPose,
                       int steps)
{
    std::vector<std::vector<double>> trajectory;
    if (!planCartesianTo(fromPose, toPose, trajectory, steps))
    {
        return false;
    }
    if (trajectory.empty())
    {
        return false;
    }

    const dxMuJoCoRobotState holdState = mSim->getRobotState();
    for (auto& point : trajectory)
    {
        for (int idx : mGripperDofIndices)
        {
            if (idx >= 0 && static_cast<size_t>(idx) < point.size() &&
                    static_cast<size_t>(idx) < holdState.jointConf.size())
            {
                point[static_cast<size_t>(idx)] = holdState.jointConf[static_cast<size_t>(idx)];
            }
        }
    }

    mTrajectory = std::move(trajectory);
    mGripperEvents.clear();
    if (mGripperHoldEnabled)
    {
        mGripperEvents.emplace_back(0, mGripperHoldRatio);
    }

    QEventLoop loop;
    const QMetaObject::Connection conn = connect(this, &demo::trajectoryFinished, &loop, &QEventLoop::quit);
    startTrajectoryPlayback();
    loop.exec();
    disconnect(conn);
    return true;
}

void demo::waitSteps(int holdSteps)
{
    const dxMuJoCoRobotState holdState = mSim->getRobotState();
    if (holdState.jointConf.empty())
    {
        return;
    }

    mTrajectory.assign(static_cast<size_t>(std::max(1, holdSteps)), holdState.jointConf);
    mGripperEvents.clear();
    if (mGripperHoldEnabled)
    {
        mGripperEvents.emplace_back(0, mGripperHoldRatio);
    }

    QEventLoop loop;
    const QMetaObject::Connection conn = connect(this, &demo::trajectoryFinished, &loop, &QEventLoop::quit);
    startTrajectoryPlayback();
    loop.exec();
    disconnect(conn);
}

void demo::setGripperHoldRatio(double ratio)
{
    mGripperHoldEnabled = true;
    mGripperHoldRatio = ratio;
    emit gripperPositionRequested(ratio);
}




bool demo::planCartesianTo(const std::vector<double>& startPose,
                           const std::vector<double>& goalPose,
                           std::vector<std::vector<double>>& trajectory,
                           int steps)
{
    if (!mKin)
    {
        return false;
    }
    dxPlannerSimple planner(mKin.get());
    if (!planner.init())
    {
        return false;
    }
    dxPlannerSimple::Params params;
    params.steps = (steps > 0) ? steps : 80;
    params.debugPaths = true;
    params.checkCollisions = false;
    params.collisionDist = -0.001;
    planner.setParams(params);

    const std::vector<double> seed = mSim->getRobotState().jointConf;
    if (seed.empty())
    {
        return false;
    }

    return planner.planCartesian(startPose, goalPose, seed, trajectory, params.steps);
}


void demo::closeGripper()
{
    setGripperHoldRatio(kGripperCloseRatio);
}

void demo::openGripper()
{
    setGripperHoldRatio(kGripperOpenRatio);
}

void demo::setGripperPosition(double ratio)
{
    emit gripperPositionRequested(ratio);
}

void demo::setEndBehavior(EndBehavior behavior)
{
    mEndBehavior = behavior;
}
