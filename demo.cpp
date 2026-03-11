#include "demo.h"

#include <iostream>
#include <QApplication>
#include <QEventLoop>
#include <QMetaObject>
#include <QThread>
#include <QTimer>

#include <mujoco/mujoco.h>

#include "dxKinMuJoCo.h"
#include "dxPlannerSimple.h"
#include "dxVision.h"

namespace
{
	constexpr double kGripperOpenRatio = 0.0;
	constexpr double kGripperCloseRatio = 1.0;
	constexpr double kTaskSpaceFovyDeg = 25.0;
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
	mInterface->setControlRateHz(250.0);
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
	const BrickZInfo cubeInfo = computeBrickZInfo("lego_brick", "lego_brick_col");
	if (!cubeInfo.valid)
	{
		return;
	}

	const double pregraspZ = cubeInfo.centerZ + 0.06;
	const double graspZ = cubeInfo.centerZ + 0.02;
	const double liftZ = cubeInfo.centerZ + 0.12;
	const double placeX = cubeX + 0.20;
	const double placeY = cubeY;
	const double placeZ = cubeInfo.centerZ;

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
	const BrickZInfo cubeInfo = computeBrickZInfo("lego_brick", "lego_brick_col_body");
	if (!cubeInfo.valid)
	{
		return;
	}

	const double pregraspZ = cubeInfo.centerZ + 0.03;
	const double graspZ = cubeInfo.centerZ;
	const double liftZ = cubeInfo.centerZ + 0.12;
	const double placeX = cubeX + 0.20;
	const double placeY = cubeY;
	const double placeZ = cubeInfo.centerZ;

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
	const double pregraspClear = 0.04;
	const double seatDepth = 0.005;

	auto makePose = [&](double x, double y, double z)
	{
		return std::vector<double>
		{
			x, y, z,
				state.eeQuat[0], state.eeQuat[1], state.eeQuat[2], state.eeQuat[3]
		};
	};

	auto pickAndPlace = [&](const std::string& pickBody,
							const std::string& pickGeom,
							const std::vector<double>& placePose,
							double placeCenterZ) -> bool
	{
		std::vector<double> pickPose = mInterface->getBodyPoseByName(pickBody);
		if (pickPose.size() < 3)
		{
			return false;
		}
		const BrickZInfo pickInfo = computeBrickZInfo(pickBody, pickGeom);
		if (!pickInfo.valid)
		{
			return false;
		}

		const double pickX = pickPose[0];
		const double pickY = pickPose[1];
		const std::vector<double> startPose =
		{
			state.eePos[0], state.eePos[1], state.eePos[2],
			state.eeQuat[0], state.eeQuat[1], state.eeQuat[2], state.eeQuat[3]
		};
		const std::vector<double> pregraspPose = makePose(pickX, pickY, pickInfo.centerZ + pregraspClear);
		const std::vector<double> graspPose = makePose(pickX, pickY, pickInfo.centerZ + 0.005);
		const std::vector<double> liftPose = makePose(pickX, pickY, pickInfo.centerZ + liftOffset);
		const std::vector<double> preplacePose = makePose(placePose[0], placePose[1], placeCenterZ + liftOffset);
		const std::vector<double> seatPose = makePose(placePose[0], placePose[1], placeCenterZ - seatDepth);

		setToolRatioFull(kGripperOpenRatio);
		waitSteps(10);

		if (!sendRobotTo(startPose, pregraspPose, 40))
		{
			return false;
		}
		if (!sendRobotTo(pregraspPose, graspPose, 80))
		{
			return false;
		}

		waitSteps(10);
		setToolRatioFull(kGripperCloseRatio);
		waitSteps(20);

		if (!sendRobotTo(graspPose, liftPose, 80))
		{
			return false;
		}
		if (!sendRobotTo(liftPose, preplacePose, 40))
		{
			return false;
		}
		if (!sendRobotTo(preplacePose, placePose, 80))
		{
			return false;
		}
		if (seatDepth > 0.0)
		{
			if (!sendRobotTo(placePose, seatPose, 100))
			{
				return false;
			}
		}

		waitSteps(10);
		setToolRatioFull(kGripperOpenRatio);
		waitSteps(30);

		sendRobotTo(placePose, preplacePose, 60);
		return true;
	};

	std::vector<double> redPose = mInterface->getBodyPoseByName("lego_brick");
	if (redPose.size() < 3)
	{
		return;
	}

	BrickZInfo redInfo = computeBrickZInfo("lego_brick", "lego_brick_col");
	if (!redInfo.valid)
	{
		return;
	}
	const std::vector<double> redPlacePose = makePose(redPose[0], redPose[1], redInfo.centerZ);
	// if (!pickAndPlace("lego_brick", "lego_brick_col", redPlacePose, redInfo.centerZ))
	// {
	//     return;
	// }

	redPose = mInterface->getBodyPoseByName("lego_brick");
	if (redPose.size() < 3)
	{
		return;
	}
	redInfo = computeBrickZInfo("lego_brick", "lego_brick_col");
	if (!redInfo.valid)
	{
		return;
	}

	const BrickZInfo yellowInfo = computeBrickZInfo("lego_brick_yellow", "lego_brick_yellow_col");
	if (!yellowInfo.valid)
	{
		return;
	}
	const BrickZInfo blueInfo = computeBrickZInfo("lego_brick_blue", "lego_brick_blue_col");
	if (!blueInfo.valid)
	{
		return;
	}
	const BrickZInfo greenInfo = computeBrickZInfo("lego_brick_green", "lego_brick_green_col");
	if (!greenInfo.valid)
	{
		return;
	}
	const BrickZInfo whiteInfo = computeBrickZInfo("lego_brick_white", "lego_brick_white_col");
	if (!whiteInfo.valid)
	{
		return;
	}

	const double seatOffset = 0.0045;
	const double stackCenterZ = redInfo.topZ + 0.5 * yellowInfo.height - seatOffset;
	const std::vector<double> stackPose = makePose(redPose[0], redPose[1], stackCenterZ);

	const double blueX = redPose[0] - 0.042;
	const double blueY = redPose[1];
	const double blueCenterZ = blueInfo.centerZ + 0.01;
	const std::vector<double> bluePose = makePose(blueX, blueY, blueCenterZ);


	const double yellowCenterZ = redInfo.topZ + 0.5 * yellowInfo.height;
	const std::vector<double> yellowPose = makePose(redPose[0], redPose[1], yellowCenterZ);


	const double greenCenterZ = blueInfo.topZ + 0.5 * greenInfo.height;
	const std::vector<double> greenPose = makePose(blueX, blueY, greenCenterZ);


	const double whiteX = 0.5 * (redPose[0] + blueX);
	const double whiteY = 0.5 * (redPose[1] + blueY);
	const double topZ = std::max(redInfo.topZ + yellowInfo.height, greenInfo.topZ);
	// const double whiteCenterZ = topZ + 0.5 * whiteInfo.height;
	const double whiteCenterZ = greenCenterZ + (whiteInfo.height * 0.75);
	const std::vector<double> whitePose = makePose(whiteX, whiteY, whiteCenterZ);

	pickAndPlace("lego_brick_blue", "lego_brick_blue_col", bluePose, blueCenterZ);
	//pickAndPlace("lego_brick_yellow", "lego_brick_yellow_col", stackPose, stackCenterZ);
	pickAndPlace("lego_brick_yellow", "lego_brick_yellow_col", yellowPose, yellowCenterZ);
	pickAndPlace("lego_brick_green", "lego_brick_green_col", greenPose, greenCenterZ);
	pickAndPlace("lego_brick_white", "lego_brick_white_col", whitePose, whiteCenterZ);
	// pickAndPlace("lego_brick_green", "lego_brick_green_col", greenPose, greenCenterZ);
	// pickAndPlace("lego_brick_white", "lego_brick_white_col", whitePose, whiteCenterZ);
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
		mujoparams.fovyDeg = kTaskSpaceFovyDeg;

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

demo::BrickZInfo demo::computeBrickZInfo(const std::string& bodyName, const std::string& geomName) const
{
	BrickZInfo info;
	if (!mInterface || !mInterface->model())
	{
		return info;
	}

	mjModel* model = mInterface->model();
	const std::vector<double> bodyPose = mInterface->getBodyPoseByName(bodyName);
	if (bodyPose.size() < 3)
	{
		return info;
	}
	const int gid = mj_name2id(model, mjOBJ_GEOM, geomName.c_str());
	if (gid < 0 || gid >= model->ngeom)
	{
		return info;
	}

	double minLocal[3] = { 0.0, 0.0, 0.0 };
	double maxLocal[3] = { 0.0, 0.0, 0.0 };
	double scale[3] = { 1.0, 1.0, 1.0 };

	const int geomType = model->geom_type[gid];
	if (geomType == mjGEOM_MESH)
	{
		const int meshId = model->geom_dataid[gid];
		if (meshId >= 0 && meshId < model->nmesh)
		{
			scale[0] = model->mesh_scale[3 * meshId];
			scale[1] = model->mesh_scale[3 * meshId + 1];
			scale[2] = model->mesh_scale[3 * meshId + 2];
		}
		minLocal[0] = 0.0;
		minLocal[1] = 0.0;
		minLocal[2] = 0.0;
		maxLocal[0] = 0.016;
		maxLocal[1] = 0.011311;
		maxLocal[2] = 0.016;
	}
	else if (geomType == mjGEOM_BOX)
	{
		const double sx = model->geom_size[3 * gid];
		const double sy = model->geom_size[3 * gid + 1];
		const double sz = model->geom_size[3 * gid + 2];
		minLocal[0] = -sx;
		minLocal[1] = -sy;
		minLocal[2] = -sz;
		maxLocal[0] = sx;
		maxLocal[1] = sy;
		maxLocal[2] = sz;
	}
	else if (geomType == mjGEOM_CYLINDER)
	{
		const double radius = model->geom_size[3 * gid];
		const double halfHeight = model->geom_size[3 * gid + 1];
		minLocal[0] = -radius;
		minLocal[1] = -radius;
		minLocal[2] = -halfHeight;
		maxLocal[0] = radius;
		maxLocal[1] = radius;
		maxLocal[2] = halfHeight;
	}
	const double* q = model->geom_quat + 4 * gid;

	auto rotateVec = [&](double x, double y, double z, double& rx, double& ry, double& rz)
	{
		const double qw = q[0];
		const double qx = q[1];
		const double qy = q[2];
		const double qz = q[3];
		const double tx = 2.0 * (qy * z - qz * y);
		const double ty = 2.0 * (qz * x - qx * z);
		const double tz = 2.0 * (qx * y - qy * x);
		rx = x + qw * tx + (qy * tz - qz * ty);
		ry = y + qw * ty + (qz * tx - qx * tz);
		rz = z + qw * tz + (qx * ty - qy * tx);
	};

	double minZ = 1e9;
	double maxZ = -1e9;
	for (int xi = 0; xi < 2; ++xi)
	{
		for (int yi = 0; yi < 2; ++yi)
		{
			for (int zi = 0; zi < 2; ++zi)
			{
				const double x = (xi == 0 ? minLocal[0] : maxLocal[0]) * scale[0];
				const double y = (yi == 0 ? minLocal[1] : maxLocal[1]) * scale[1];
				const double z = (zi == 0 ? minLocal[2] : maxLocal[2]) * scale[2];
				double rx = 0.0;
				double ry = 0.0;
				double rz = 0.0;
				rotateVec(x, y, z, rx, ry, rz);
				if (rz < minZ)
				{
					minZ = rz;
				}
				if (rz > maxZ)
				{
					maxZ = rz;
				}
			}
		}
	}

	const double geomPosZ = model->geom_pos[3 * gid + 2];
	info.bottomZ = bodyPose[2] + geomPosZ + minZ;
	info.topZ = bodyPose[2] + geomPosZ + maxZ;
	info.centerZ = 0.5 * (info.bottomZ + info.topZ);
	info.height = info.topZ - info.bottomZ;
	info.valid = true;
	return info;
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

