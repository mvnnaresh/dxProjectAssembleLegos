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
constexpr double kGripperCloseRatio = 1.0;
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

    emit updateUIMessage("Demo Initalised!!");
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

//std::vector<double> demo::extractArmDof(const std::vector<double>& dofQpos) const
//{
//    std::vector<double> out;
//    out.reserve(mArmDofIndices.size());
//    for (int idx : mArmDofIndices)
//    {
//        if (idx >= 0 && static_cast<size_t>(idx) < dofQpos.size())
//        {
//            out.push_back(dofQpos[static_cast<size_t>(idx)]);
//        }
//    }
//    return out;
//}
//
//std::vector<double> demo::expandArmDof(const std::vector<double>& baseDof,
//                                       const std::vector<double>& armDof) const
//{
//    std::vector<double> out = baseDof;
//    const size_t limit = std::min(mArmDofIndices.size(), armDof.size());
//    for (size_t i = 0; i < limit; ++i)
//    {
//        const int idx = mArmDofIndices[i];
//        if (idx >= 0 && static_cast<size_t>(idx) < out.size())
//        {
//            out[static_cast<size_t>(idx)] = armDof[i];
//        }
//    }
//    return out;
//}

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
        return;

    dxKinMuJoCo kin(mSim->model(), mSim->data());
    dxPlannerSimple planner(&kin);

    if (!planner.init())
        return;

    //!------ Set Planner Params
    dxPlannerSimple::Params params;
    params.steps = 200;
    params.debugPaths = true;
    params.checkCollisions = true;
    params.collisionDist = -0.001;
    params.showTrajectoryCurve = true;
    planner.setParams(params);


    //!----- set planner start Position
    std::vector<double> start = mSim->getRobotState().jointConf;
    planner.setStart(start);

    //!----- set Planner Goal Position
    double delta = 25.0 * 3.141592653589793 / 180.0;
    std::vector<double> goal = start;
    for (int i = 0; i < 6; i++)
        goal[i] += delta;
    planner.setGoal(goal);


    //!----- Solve for a path
    if (!planner.solve())
    {
        emit updateUIMessage("Simple Joint Planner Failed!!");
        return;
    }
    const std::vector<std::vector<double>>& path = planner.getPath();
    emit updateUIMessage("Simple Planner successful - Path size: " + std::to_string(path.size()));

    //!-----  Build Trajectory
    const std::vector<std::vector<double>> trajectory = planner.buildTrajectory(path);

    mTrajectory = trajectory;
    if (params.showTrajectoryCurve)
    {
        emit drawTrajectory(planner.getTrajAs3DPoints(mTrajectory));
    }

    //!----- Simulate Trajectory
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

    const dxMuJoCoRobotState state = mSim->getRobotState();
    if (state.jointConf.empty())
    {
        return;
    }

    std::vector<double> cubePose = mSim->getBodyPoseByName("lego_brick");
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

    waitSteps(20);

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

void demo::testCamera()
{
    emit cameraStreamRequested(true);
    emit cameraPointCloudRequested();
    emit updateUIMessage("Camera RGB + point cloud requested.");
}

void demo::testCamera3D()
{
    if (!mViewer)
    {
        emit updateUIMessage("No viewer available for camera capture.");
        return;
    }

    mViewer->runWithContext([this]()
    {
        dxVision camA;
        MuJoCoCameraParams mujoparams;
        mujoparams.model = mViewer->model();
        mujoparams.data = mViewer->data();
        mujoparams.opt = mViewer->mjvOptionPtr();
        mujoparams.ctx = mViewer->mjrContextPtr();
        mujoparams.cameraName = "scene_cam";
        mujoparams.baseBodyName = "base";
        mujoparams.width = 640;
        mujoparams.height = 480;

        if (!camA.initMujocoCamera(mujoparams))
        {
            return;
        }
        CloudPtr cloud = camA.acquireMujocoPointCloud();
        if (cloud && !cloud->points.empty())
        {
            camA.viewPointCloud(cloud, "camera_cloud", "Camera Point Cloud", 2);
        }
    });

    emit updateUIMessage("Camera point cloud requested.");
}

void demo::setViewer(dxMuJoCoRobotViewer* viewer)
{
    mViewer = viewer;
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
