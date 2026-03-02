#include "demo.h"

#include <iostream>
#include <QApplication>
#include <QEventLoop>
#include <QMetaObject>
#include <QThread>
#include <QTimer>

#include "dxKinMuJoCo.h"
#include "dxPlannerSimple.h"
#include "dxVision.h"

namespace
{
	constexpr double kGripperOpenRatio = 0.0;
	constexpr double kGripperCloseRatio = 1.0;
}

demo::demo(dxMujocoInterface* interfacePtr, QObject* parent)
	: QObject(parent), mInterface(interfacePtr)
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
			if (mInterface)
			{
				if (mUseFullCtrl && !mFullCtrlTargets.empty())
				{
					const size_t limit = std::min({ joints.size(),
													static_cast<size_t>(mArmDofCount),
													mFullCtrlTargets.size()
												  });
					for (size_t i = 0; i < limit; ++i)
					{
						mFullCtrlTargets[i] = joints[i];
					}
					mInterface->setCtrlTargetsFromFullJointPositions(mFullCtrlTargets);
				}
				else
				{
					mInterface->setCtrlTargetsFromJointPositions(joints);
				}
			}
			return;
		}

		const std::vector<double>& joints = mTrajectory[mTrajectoryIndex++];
		if (mInterface)
		{
			if (mUseFullCtrl && !mFullCtrlTargets.empty())
			{
				const size_t limit = std::min({ joints.size(),
												static_cast<size_t>(mArmDofCount),
												mFullCtrlTargets.size()
											  });
				for (size_t i = 0; i < limit; ++i)
				{
					mFullCtrlTargets[i] = joints[i];
				}
				mInterface->setCtrlTargetsFromFullJointPositions(mFullCtrlTargets);
			}
			else
			{
				mInterface->setCtrlTargetsFromJointPositions(joints);
			}
		}
	});

}

// Initialize kinematics helper after the simulator has a model loaded.
bool demo::init()
{
	if (!mInterface || !mInterface->model())
	{
		return false;
	}

	mKin = std::make_unique<dxKinMuJoCo>(mInterface->model(), mInterface->data());

	mInterface->setArmDofCount(6);
	mArmDofCount = 6;
	mFullCtrlTargets = mInterface->getRobotState().jointConf;
	mToolRatio = kGripperOpenRatio;

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
	if (!mTrajectoryTimer->isActive())
	{
		mTrajectoryTimer->start();
	}
}


// Validate FK/IK by computing the current pose and attempting to solve IK back to it.
void demo::testKinematics()
{
	if (!mInterface || !mInterface->model())
	{
		return;
	}

	dxKinMuJoCo kin(mInterface->model(), mInterface->data());
	const dxMuJoCoRobotState state = mInterface->getRobotState();

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
	if (!mInterface || !mInterface->model())
		return;

	dxKinMuJoCo kin(mInterface->model(), mInterface->data());
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
	std::vector<double> start = mInterface->getRobotState().jointConf;
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
	if (!mInterface || !mInterface->model())
	{
		return;
	}

	dxKinMuJoCo kin(mInterface->model(), mInterface->data());
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

	const dxMuJoCoRobotState state = mInterface->getRobotState();
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
		mTrajectory.push_back(point);
	}
	emit drawTrajectory(planner.getTrajAs3DPoints(mTrajectory));
	startTrajectoryPlayback();
}


void demo::testPickAndPlace()
{
	if (!mInterface || !mInterface->model() || !mKin)
	{
		return;
	}

	const dxMuJoCoRobotState state = mInterface->getRobotState();
	if (state.jointConf.empty())
	{
		return;
	}

	std::vector<double> cubePose = mInterface->getBodyPoseByName("lego_brick");
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

	waitSteps(40);
	openGripper();
	waitSteps(40);

	sendRobotTo(placePose, preplacePose, 40);
	std::cout << "[testPickAndPlace3] returned to preplace" << std::endl;
}

void demo::testNewPickAndPlace()
{
	if (!mInterface || !mInterface->model() || !mKin)
	{
		return;
	}

	const dxMuJoCoRobotState state = mInterface->getRobotState();
	if (state.jointConf.empty())
	{
		return;
	}

	std::vector<double> cubePose = mInterface->getBodyPoseByName("lego_brick");
	const double cubeX = cubePose[0];
	const double cubeY = cubePose[1];
	const double cubeZ = cubePose[2];

	const double pregraspZ = cubeZ + 0.10;
	const double graspZ = cubeZ + 0.02;
	const double liftZ = cubeZ + 0.12;
	const double placeX = cubeX + 0.20;
	const double placeY = cubeY;
	const double placeZ = cubeZ + 0.02;

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

	const int sleepMs = 5; //latency

	setToolRatioFull(kGripperOpenRatio);
	waitStepsBlockingFull(10, sleepMs);

	sendRobotToBlockingFull(startPose, pregraspPose, 60, sleepMs);
	sendRobotToBlockingFull(pregraspPose, graspPose, 40, sleepMs);

	waitStepsBlockingFull(10, sleepMs);

	setToolRatioFull(kGripperCloseRatio);
	waitStepsBlockingFull(20, sleepMs);

	sendRobotToBlockingFull(graspPose, pregraspPose, 40, sleepMs);
	sendRobotToBlockingFull(pregraspPose, preplacePose, 60, sleepMs);
	sendRobotToBlockingFull(preplacePose, placePose, 40, sleepMs);

	waitStepsBlockingFull(10, sleepMs);
	setToolRatioFull(kGripperOpenRatio);
	waitStepsBlockingFull(40, sleepMs);

	sendRobotToBlockingFull(placePose, preplacePose, 40, sleepMs);
}

void demo::testLegoAssembly()
{
	if (!mInterface || !mInterface->model() || !mKin)
	{
		return;
	}

	const dxMuJoCoRobotState state = mInterface->getRobotState();
	if (state.jointConf.empty())
	{
		return;
	}

	const int sleepMs = 20;
	const double liftOffset = 0.12;
	const double pregraspOffset = 0.10;
	const double graspOffset = 0.02;
	const double stackOffset = 0.06;

	auto makePose = [&](double x, double y, double z)
	{
		return std::vector<double>
		{
			x, y, z,
				state.eeQuat[0], state.eeQuat[1], state.eeQuat[2], state.eeQuat[3]
		};
	};

	auto pickAndPlace = [&](const std::string& pickBody,
							const std::vector<double>& placePose) -> bool
	{
		std::vector<double> pickPose = mInterface->getBodyPoseByName(pickBody);
		if (pickPose.size() < 3)
		{
			return false;
		}

		const double pickX = pickPose[0];
		const double pickY = pickPose[1];
		const double pickZ = pickPose[2];
		const std::vector<double> startPose =
		{
			state.eePos[0], state.eePos[1], state.eePos[2],
			state.eeQuat[0], state.eeQuat[1], state.eeQuat[2], state.eeQuat[3]
		};
		const std::vector<double> pregraspPose = makePose(pickX, pickY, pickZ + pregraspOffset);
		const std::vector<double> graspPose = makePose(pickX, pickY, pickZ + graspOffset);
		const std::vector<double> liftPose = makePose(pickX, pickY, pickZ + liftOffset);
		const std::vector<double> preplacePose = makePose(placePose[0], placePose[1], placePose[2] + liftOffset);

		setToolRatioFull(kGripperOpenRatio);
		waitStepsBlockingFull(10, sleepMs);

		if (!sendRobotToBlockingFull(startPose, pregraspPose, 60, sleepMs))
		{
			return false;
		}
		if (!sendRobotToBlockingFull(pregraspPose, graspPose, 40, sleepMs))
		{
			return false;
		}

		waitStepsBlockingFull(10, sleepMs);
		setToolRatioFull(kGripperCloseRatio);
		waitStepsBlockingFull(20, sleepMs);

		if (!sendRobotToBlockingFull(graspPose, liftPose, 40, sleepMs))
		{
			return false;
		}
		if (!sendRobotToBlockingFull(liftPose, preplacePose, 60, sleepMs))
		{
			return false;
		}
		if (!sendRobotToBlockingFull(preplacePose, placePose, 40, sleepMs))
		{
			return false;
		}

		waitStepsBlockingFull(10, sleepMs);
		setToolRatioFull(kGripperOpenRatio);
		waitStepsBlockingFull(30, sleepMs);

		sendRobotToBlockingFull(placePose, preplacePose, 80, sleepMs);
		return true;
	};

	std::vector<double> redPose = mInterface->getBodyPoseByName("lego_brick");
	if (redPose.size() < 3)
	{
		return;
	}

	const std::vector<double> redPlacePose = makePose(redPose[0], redPose[1], redPose[2] + graspOffset);
	// if (!pickAndPlace("lego_brick", redPlacePose))
	// {
	// 	return;
	// }

	const std::vector<double> stackPose = makePose(redPose[0], redPose[1], redPose[2] + stackOffset);
	pickAndPlace("lego_brick_yellow", stackPose);
}

void demo::testCamera()
{
	if (!mInterface)
	{
		emit updateUIMessage("No MuJoCo interface available for camera capture.");
		return;
	}

	dxMuJoCoRobotViewer* viewer = mInterface->viewer();
	if (!viewer)
	{
		emit updateUIMessage("No viewer available for camera capture.");
		return;
	}

	viewer->setCameraStreamEnabled(true);
	viewer->requestPointCloudCapture();
	emit updateUIMessage("Camera RGB + point cloud requested.");
}

void demo::testCamera3D()
{
	if (!mInterface)
	{
		emit updateUIMessage("No MuJoCo interface available for camera capture.");
		return;
	}

	dxMuJoCoRobotViewer* viewer = mInterface->viewer();
	if (!viewer)
	{
		emit updateUIMessage("No viewer available for camera capture.");
		return;
	}

	viewer->runWithContext([this, viewer]()
	{
		dxVision camA;
		MuJoCoCameraParams mujoparams;
		mujoparams.model = viewer->model();
		mujoparams.data = viewer->data();
		mujoparams.opt = viewer->mjvOptionPtr();
		mujoparams.ctx = viewer->mjrContextPtr();
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

	mTrajectory = std::move(trajectory);
	QEventLoop loop;
	const QMetaObject::Connection conn = connect(this, &demo::trajectoryFinished, &loop, &QEventLoop::quit);
	startTrajectoryPlayback();
	loop.exec();
	disconnect(conn);
	return true;
}

bool demo::sendRobotToBlocking(const std::vector<double>& fromPose,
							   const std::vector<double>& toPose,
							   int steps,
							   int sleepMs)
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

	if (sleepMs < 0)
	{
		sleepMs = 0;
	}

	for (const auto& joints : trajectory)
	{
		if (mInterface)
		{
			mInterface->setCtrlTargetsFromJointPositions(joints);
		}
		QApplication::processEvents();
		if (sleepMs > 0)
		{
			QThread::msleep(static_cast<unsigned long>(sleepMs));
		}
	}
	return true;
}

bool demo::sendRobotToBlockingFull(const std::vector<double>& fromPose,
								   const std::vector<double>& toPose,
								   int steps,
								   int sleepMs)
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
	if (!mInterface)
	{
		return false;
	}
	if (sleepMs < 0)
	{
		sleepMs = 0;
	}

	for (const auto& joints : trajectory)
	{
		const std::vector<double> fullTargets =
			mInterface->buildFullJointTargets(joints, mArmDofCount, mToolRatio);
		if (!fullTargets.empty())
		{
			mInterface->setCtrlTargetsFromFullJointPositions(fullTargets);
		}
		QApplication::processEvents();
		if (sleepMs > 0)
		{
			QThread::msleep(static_cast<unsigned long>(sleepMs));
		}
	}
	return true;
}

void demo::waitSteps(int holdSteps)
{
	const dxMuJoCoRobotState holdState = mInterface->getRobotState();
	if (holdState.jointConf.empty())
	{
		return;
	}

	mTrajectory.assign(static_cast<size_t>(std::max(1, holdSteps)), holdState.jointConf);
	QEventLoop loop;
	const QMetaObject::Connection conn = connect(this, &demo::trajectoryFinished, &loop, &QEventLoop::quit);
	startTrajectoryPlayback();
	loop.exec();
	disconnect(conn);
}

void demo::waitStepsBlocking(int holdSteps, int sleepMs)
{
	if (!mInterface)
	{
		return;
	}
	if (sleepMs < 0)
	{
		sleepMs = 0;
	}

	const int count = std::max(1, holdSteps);
	for (int i = 0; i < count; ++i)
	{
		QApplication::processEvents();
		if (sleepMs > 0)
		{
			QThread::msleep(static_cast<unsigned long>(sleepMs));
		}
	}
}

void demo::waitStepsBlockingFull(int holdSteps, int sleepMs)
{
	if (!mInterface)
	{
		return;
	}
	const dxMuJoCoRobotState holdState = mInterface->getRobotState();
	if (holdState.jointConf.empty())
	{
		return;
	}
	if (sleepMs < 0)
	{
		sleepMs = 0;
	}

	std::vector<double> armHold = holdState.jointConf;
	if (armHold.size() > static_cast<size_t>(mArmDofCount))
	{
		armHold.resize(static_cast<size_t>(mArmDofCount));
	}

	const int count = std::max(1, holdSteps);
	for (int i = 0; i < count; ++i)
	{
		const std::vector<double> fullTargets =
			mInterface->buildFullJointTargets(armHold, mArmDofCount, mToolRatio);
		if (!fullTargets.empty())
		{
			mInterface->setCtrlTargetsFromFullJointPositions(fullTargets);
		}
		QApplication::processEvents();
		if (sleepMs > 0)
		{
			QThread::msleep(static_cast<unsigned long>(sleepMs));
		}
	}
}

void demo::setToolRatioFull(double ratio)
{
	if (ratio < 0.0)
	{
		ratio = 0.0;
	}
	else if (ratio > 1.0)
	{
		ratio = 1.0;
	}
	mToolRatio = ratio;

	if (!mInterface)
	{
		return;
	}
	const dxMuJoCoRobotState state = mInterface->getRobotState();
	if (state.jointConf.empty())
	{
		return;
	}
	std::vector<double> armHold = state.jointConf;
	if (armHold.size() > static_cast<size_t>(mArmDofCount))
	{
		armHold.resize(static_cast<size_t>(mArmDofCount));
	}
	const std::vector<double> fullTargets =
		mInterface->buildFullJointTargets(armHold, mArmDofCount, mToolRatio);
	if (!fullTargets.empty())
	{
		mInterface->setCtrlTargetsFromFullJointPositions(fullTargets);
	}
}

bool demo::runFullTrajectory(const std::vector<std::vector<double>>& armTrajectory)
{
	if (!mInterface || armTrajectory.empty())
	{
		return false;
	}
	if (mFullCtrlTargets.empty())
	{
		mFullCtrlTargets = mInterface->getRobotState().jointConf;
	}
	if (mFullCtrlTargets.empty())
	{
		return false;
	}

	mTrajectory = armTrajectory;
	mUseFullCtrl = true;

	QEventLoop loop;
	const QMetaObject::Connection conn = connect(this, &demo::trajectoryFinished, &loop, &QEventLoop::quit);
	startTrajectoryPlayback();
	loop.exec();
	disconnect(conn);

	mUseFullCtrl = false;
	return true;
}

void demo::waitFullSteps(int holdSteps)
{
	if (!mInterface)
	{
		return;
	}
	if (mFullCtrlTargets.empty())
	{
		mFullCtrlTargets = mInterface->getRobotState().jointConf;
	}
	if (mFullCtrlTargets.empty())
	{
		return;
	}

	std::vector<double> armHold;
	const size_t limit = std::min(static_cast<size_t>(mArmDofCount), mFullCtrlTargets.size());
	armHold.assign(mFullCtrlTargets.begin(), mFullCtrlTargets.begin() + static_cast<long long>(limit));
	const int clamped = std::max(1, holdSteps);
	std::vector<std::vector<double>> armTrajectory(static_cast<size_t>(clamped), armHold);
	runFullTrajectory(armTrajectory);
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

	const std::vector<double> seed = mInterface->getRobotState().jointConf;
	if (seed.empty())
	{
		return false;
	}

	return planner.planCartesian(startPose, goalPose, seed, trajectory, params.steps);
}


void demo::closeGripper()
{
	setGripperPosition(kGripperCloseRatio);
}

void demo::openGripper()
{
	setGripperPosition(kGripperOpenRatio);
}

void demo::setGripperPosition(double ratio)
{
	if (mInterface)
	{
		mInterface->setGripperPosition(ratio);
	}
}

void demo::setEndBehavior(EndBehavior behavior)
{
	mEndBehavior = behavior;
}

