// MuJoCo simulator wrapper: load, drive control loop, and emit state updates.
#include "dxMuJoCoRobotSimulator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace
{
// Clear generalized velocities to remove residual motion on startup/reset.
void zero_qvel(mjModel* model, mjData* data)
{
    if (!model || !data)
    {
        return;
    }
    for (int i = 0; i < model->nv; ++i)
    {
        data->qvel[i] = 0.0;
    }
}
}

dxMuJoCoRobotSimulator::dxMuJoCoRobotSimulator(QObject* parent)
    : QObject(parent)
{
}

dxMuJoCoRobotSimulator::~dxMuJoCoRobotSimulator()
{
    stop();
    shutdown();
}

void dxMuJoCoRobotSimulator::setControlRateHz(double rateHz)
{
    if (rateHz <= 0.0)
    {
        return;
    }
    mControlRateHz = rateHz;
    if (mTimer && mTimer->isActive())
    {
        const int intervalMs = std::max(1, static_cast<int>(1000.0 / mControlRateHz));
        mTimer->setInterval(intervalMs);
    }
}

double dxMuJoCoRobotSimulator::controlRateHz() const
{
    return mControlRateHz;
}

mjModel* dxMuJoCoRobotSimulator::model() const
{
    return mModel;
}

mjData* dxMuJoCoRobotSimulator::data() const
{
    return mData;
}

dxMuJoCoRobotState dxMuJoCoRobotSimulator::getRobotState() const
{
    std::lock_guard<std::mutex> lock(mStateMutex);
    return mState;
}

void dxMuJoCoRobotSimulator::setCtrlTargets(const std::vector<double>& targets)
{
    std::lock_guard<std::mutex> lock(mTargetMutex);
    mPendingCtrlTargets = targets;
    mHasCtrlTargets = true;
    mHoldCtrlTargets = targets;
    mHoldMode = HoldMode::HoldCtrlTargets;
}

void dxMuJoCoRobotSimulator::setCtrlByName(const std::string& actuatorName, double value)
{
    if (actuatorName.empty())
    {
        return;
    }
    std::lock_guard<std::mutex> lock(mTargetMutex);
    mPendingNamedCtrls.emplace_back(actuatorName, value);
}

void dxMuJoCoRobotSimulator::setCtrlTargetsFromJointPositions(const std::vector<double>& jointPositions)
{
    if (!mModel || jointPositions.empty())
    {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mTargetMutex);
        mHoldJointTargets = jointPositions;
        mHoldMode = HoldMode::HoldJointTargets;
    }

    std::vector<int> jointOrder(static_cast<size_t>(mModel->njnt), -1);
    int orderIndex = 0;
    for (int jid = 0; jid < mModel->njnt; ++jid)
    {
        const int type = mModel->jnt_type[jid];
        if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
        {
            continue;
        }
        jointOrder[jid] = orderIndex++;
    }

    std::vector<double> targets(static_cast<size_t>(mModel->nu), 0.0);
    for (int aid = 0; aid < mModel->nu; ++aid)
    {
        if (mModel->actuator_trntype[aid] != mjTRN_JOINT)
        {
            continue;
        }
        const int jid = mModel->actuator_trnid[2 * aid];
        if (jid < 0 || jid >= mModel->njnt)
        {
            continue;
        }
        const int idx = jointOrder[jid];
        if (idx < 0 || idx >= static_cast<int>(jointPositions.size()))
        {
            continue;
        }
        targets[static_cast<size_t>(aid)] = jointPositions[static_cast<size_t>(idx)];
    }

    std::lock_guard<std::mutex> lock(mTargetMutex);
    mPendingCtrlTargets = targets;
    mHasCtrlTargets = true;
}

void dxMuJoCoRobotSimulator::setJointPositions(const std::vector<double>& jointPositions)
{
    if (jointPositions.empty())
    {
        return;
    }
    std::lock_guard<std::mutex> lock(mTargetMutex);
    mPendingQpos = jointPositions;
    mHasPendingQpos = true;
    mHoldJointTargets = jointPositions;
    mHoldMode = HoldMode::HoldJointTargets;
}

void dxMuJoCoRobotSimulator::setPdGains(double kp, double kd)
{
    if (kp < 0.0 || kd < 0.0)
    {
        return;
    }
    mPdKp = kp;
    mPdKd = kd;
}

void dxMuJoCoRobotSimulator::enablePdHold(bool enabled)
{
    mEnablePdHold = enabled;
}

void dxMuJoCoRobotSimulator::enableHardLock(bool enabled)
{
    mEnableHardLock = enabled;
}

void dxMuJoCoRobotSimulator::lockCurrentPose()
{
    if (!mModel || !mData)
    {
        return;
    }
    const std::vector<double> joints = extractJointPositions();
    if (joints.empty())
    {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mTargetMutex);
        mHoldJointTargets = joints;
        mHoldMode = HoldMode::HoldJointTargets;
    }
    if (mEnableHardLock)
    {
        applyJointPositionsDirect(joints);
        zero_qvel(mModel, mData);
        mj_forward(mModel, mData);
    }
}

void dxMuJoCoRobotSimulator::closeGripper()
{
    if (!mModel || !mData || mModel->nu <= 0)
    {
        return;
    }

    std::vector<double> targets;
    targets.assign(mData->ctrl, mData->ctrl + mModel->nu);

    const std::vector<int> tendonActuators = getTendonActuatorIndices();
    if (tendonActuators.empty())
    {
        return;
    }

    for (int aid : tendonActuators)
    {
        if (aid < 0 || aid >= mModel->nu)
        {
            continue;
        }
        double target = 0.0;
        if (mModel->actuator_ctrllimited[aid])
        {
            target = mModel->actuator_ctrlrange[2 * aid + 1];
        }
        else
        {
            target = 1.0;
        }
        targets[static_cast<size_t>(aid)] = target;
    }

    applyCtrlTargetsDirect(targets);
    updateStateSnapshot();

    std::lock_guard<std::mutex> lock(mTargetMutex);
    mHoldCtrlTargets = targets;
    if (mHoldMode != HoldMode::HoldJointTargets || mHoldJointTargets.empty())
    {
        mHoldMode = HoldMode::HoldCtrlTargets;
    }
}

void dxMuJoCoRobotSimulator::loadModel(const QString& modelPath)
{
    stop();
    shutdown();

    mModelPath = modelPath;
    if (mModelPath.isEmpty())
    {
        emit error("dxMuJoCoRobotSimulator: empty model path.");
        return;
    }

    char err[1024] = { 0 };
    mModel = mj_loadXML(mModelPath.toStdString().c_str(), nullptr, err, sizeof(err));
    if (!mModel)
    {
        emit error(QString("dxMuJoCoRobotSimulator: mj_loadXML failed for '%1'\nError: %2")
                   .arg(mModelPath, QString(err)));
        return;
    }

    mData = mj_makeData(mModel);
    if (!mData)
    {
        emit error("dxMuJoCoRobotSimulator: mj_makeData failed.");
        shutdown();
        return;
    }

    mPoseApplied = false;
    mPoseAppliedName.clear();
    if (!applyPoseByName("HOME_POS"))
    {
        applyPoseByName("home");
    }

    zero_qvel(mModel, mData);
    mj_forward(mModel, mData);

    {
        std::lock_guard<std::mutex> lock(mTargetMutex);
        mPendingCtrlTargets.clear();
        mHasCtrlTargets = false;
        mPendingQpos.clear();
        mHasPendingQpos = false;
        mPendingNamedCtrls.clear();
        mHoldCtrlTargets.clear();
    }

    const std::vector<double> jointPositions = extractJointPositions();
    if (!jointPositions.empty())
    {
        {
            std::lock_guard<std::mutex> lock(mTargetMutex);
            mHoldJointTargets = jointPositions;
            mHoldMode = HoldMode::HoldJointTargets;
        }
        setCtrlTargetsFromJointPositions(jointPositions);
        applyPendingTargets();
    }

    updateStateSnapshot();
    printDetails();
    emit modelLoaded(mModel);
}

void dxMuJoCoRobotSimulator::reset()
{
    if (!mModel || !mData)
    {
        return;
    }
    mj_resetData(mModel, mData);
    zero_qvel(mModel, mData);
    mj_forward(mModel, mData);
    const std::vector<double> jointPositions = extractJointPositions();
    if (!jointPositions.empty())
    {
        {
            std::lock_guard<std::mutex> lock(mTargetMutex);
            mHoldJointTargets = jointPositions;
            mHoldMode = HoldMode::HoldJointTargets;
        }
        setCtrlTargetsFromJointPositions(jointPositions);
        applyPendingTargets();
    }
    updateStateSnapshot();
    emit resetDone();
}

void dxMuJoCoRobotSimulator::start()
{
    if (mRunning)
    {
        return;
    }
    if (!mTimer)
    {
        mTimer = new QTimer(this);
        connect(mTimer, &QTimer::timeout, this, &dxMuJoCoRobotSimulator::stepLoop);
    }
    const int intervalMs = std::max(1, static_cast<int>(1000.0 / mControlRateHz));
    mTimer->setInterval(intervalMs);
    mTimer->start();
    mRunning = true;
}

void dxMuJoCoRobotSimulator::stop()
{
    if (mTimer && mTimer->isActive())
    {
        mTimer->stop();
    }
    mRunning = false;
}

void dxMuJoCoRobotSimulator::stepLoop()
{
    if (!mModel || !mData)
    {
        return;
    }

    const bool hadPending = hasPendingTargets();
    applyPendingTargets();

    if (!hadPending)
    {
        HoldMode holdMode = HoldMode::None;
        std::vector<double> holdJoints;
        std::vector<double> holdCtrls;
        {
            std::lock_guard<std::mutex> lock(mTargetMutex);
            holdMode = mHoldMode;
            holdJoints = mHoldJointTargets;
            holdCtrls = mHoldCtrlTargets;
        }

        if (holdMode == HoldMode::HoldCtrlTargets && !holdCtrls.empty())
        {
            applyCtrlTargetsDirect(holdCtrls);
        }
        else if (holdMode == HoldMode::HoldJointTargets && !holdJoints.empty())
        {
            if (mModel->nu > 0)
            {
                if (mEnableHardLock)
                {
                    applyJointPositionsDirect(holdJoints);
                    zero_qvel(mModel, mData);
                }
                else if (mEnablePdHold)
                {
                    applyPdHoldFromJointTargets(holdJoints);
                }
                else
                {
                    applyCtrlTargetsFromJointPositionsDirect(holdJoints);
                }
            }
            else
            {
                applyJointPositionsDirect(holdJoints);
            }
        }
    }

    mj_step(mModel, mData);
    updateStateSnapshot();
    emit stateUpdated();
}

bool dxMuJoCoRobotSimulator::hasPendingTargets() const
{
    std::lock_guard<std::mutex> lock(mTargetMutex);
    return mHasCtrlTargets || mHasPendingQpos || !mPendingNamedCtrls.empty();
}

void dxMuJoCoRobotSimulator::applyPendingTargets()
{
    std::vector<double> ctrlTargets;
    std::vector<double> qposTargets;
    std::vector<std::pair<std::string, double>> namedTargets;
    bool hasCtrlTargets = false;
    bool hasQposTargets = false;

    {
        std::lock_guard<std::mutex> lock(mTargetMutex);
        hasCtrlTargets = mHasCtrlTargets;
        hasQposTargets = mHasPendingQpos;
        ctrlTargets = mPendingCtrlTargets;
        qposTargets = mPendingQpos;
        namedTargets = mPendingNamedCtrls;
        mPendingNamedCtrls.clear();
        mHasCtrlTargets = false;
        mHasPendingQpos = false;
    }

    if (hasQposTargets)
    {
        applyJointPositionsDirect(qposTargets);
    }

    if (hasCtrlTargets)
    {
        applyCtrlTargetsDirect(ctrlTargets);
    }

    if (!namedTargets.empty())
    {
        for (const auto& entry : namedTargets)
        {
            const int id = mj_name2id(mModel, mjOBJ_ACTUATOR, entry.first.c_str());
            if (id >= 0 && id < mModel->nu)
            {
                mData->ctrl[id] = entry.second;
            }
        }
    }
}

void dxMuJoCoRobotSimulator::applyCtrlTargetsDirect(const std::vector<double>& targets)
{
    if (!mModel || !mData)
    {
        return;
    }
    const int count = static_cast<int>(targets.size());
    const int limit = (count < mModel->nu) ? count : mModel->nu;
    for (int i = 0; i < limit; ++i)
    {
        mData->ctrl[i] = targets[static_cast<size_t>(i)];
    }
}

void dxMuJoCoRobotSimulator::applyCtrlTargetsFromJointPositionsDirect(const std::vector<double>& jointPositions)
{
    if (!mModel || !mData || jointPositions.empty())
    {
        return;
    }

    std::vector<int> jointOrder(static_cast<size_t>(mModel->njnt), -1);
    int orderIndex = 0;
    for (int jid = 0; jid < mModel->njnt; ++jid)
    {
        const int type = mModel->jnt_type[jid];
        if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
        {
            continue;
        }
        jointOrder[jid] = orderIndex++;
    }

    for (int aid = 0; aid < mModel->nu; ++aid)
    {
        if (mModel->actuator_trntype[aid] != mjTRN_JOINT)
        {
            continue;
        }
        const int jid = mModel->actuator_trnid[2 * aid];
        if (jid < 0 || jid >= mModel->njnt)
        {
            continue;
        }
        const int idx = jointOrder[jid];
        if (idx < 0 || idx >= static_cast<int>(jointPositions.size()))
        {
            continue;
        }
        mData->ctrl[aid] = jointPositions[static_cast<size_t>(idx)];
    }
}

void dxMuJoCoRobotSimulator::applyJointPositionsDirect(const std::vector<double>& jointPositions)
{
    if (!mModel || !mData || jointPositions.empty())
    {
        return;
    }

    int jointIndex = 0;
    for (int jid = 0; jid < mModel->njnt; ++jid)
    {
        const int type = mModel->jnt_type[jid];
        if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
        {
            continue;
        }
        if (jointIndex >= static_cast<int>(jointPositions.size()))
        {
            break;
        }
        const int qposAdr = mModel->jnt_qposadr[jid];
        if (qposAdr >= 0 && qposAdr < mModel->nq)
        {
            mData->qpos[qposAdr] = jointPositions[static_cast<size_t>(jointIndex)];
        }
        ++jointIndex;
    }
    mj_forward(mModel, mData);
}

void dxMuJoCoRobotSimulator::applyPdHoldFromJointTargets(const std::vector<double>& jointPositions)
{
    if (!mModel || !mData || jointPositions.empty())
    {
        return;
    }

    std::vector<int> jointOrder(static_cast<size_t>(mModel->njnt), -1);
    int orderIndex = 0;
    for (int jid = 0; jid < mModel->njnt; ++jid)
    {
        const int type = mModel->jnt_type[jid];
        if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
        {
            continue;
        }
        jointOrder[jid] = orderIndex++;
    }

    for (int aid = 0; aid < mModel->nu; ++aid)
    {
        if (mModel->actuator_trntype[aid] != mjTRN_JOINT)
        {
            continue;
        }
        const int jid = mModel->actuator_trnid[2 * aid];
        if (jid < 0 || jid >= mModel->njnt)
        {
            continue;
        }
        const int idx = jointOrder[jid];
        if (idx < 0 || idx >= static_cast<int>(jointPositions.size()))
        {
            continue;
        }
        const int qposAdr = mModel->jnt_qposadr[jid];
        const int dofAdr = mModel->jnt_dofadr[jid];
        if (qposAdr < 0 || qposAdr >= mModel->nq || dofAdr < 0 || dofAdr >= mModel->nv)
        {
            continue;
        }

        const double target = jointPositions[static_cast<size_t>(idx)];
        const double qpos = mData->qpos[qposAdr];
        const double qvel = mData->qvel[dofAdr];
        double torque = (mPdKp * (target - qpos)) - (mPdKd * qvel);

        if (mModel->actuator_ctrllimited[aid])
        {
            const double lo = mModel->actuator_ctrlrange[2 * aid];
            const double hi = mModel->actuator_ctrlrange[2 * aid + 1];
            torque = std::min(hi, std::max(lo, torque));
        }
        mData->ctrl[aid] = torque;
    }
}

std::vector<int> dxMuJoCoRobotSimulator::getTendonActuatorIndices() const
{
    std::vector<int> indices;
    if (!mModel)
    {
        return indices;
    }
    for (int aid = 0; aid < mModel->nu; ++aid)
    {
        if (mModel->actuator_trntype[aid] == mjTRN_TENDON)
        {
            indices.push_back(aid);
        }
    }
    return indices;
}

void dxMuJoCoRobotSimulator::updateStateSnapshot()
{
    dxMuJoCoRobotState snapshot;
    if (!mModel || !mData)
    {
        return;
    }

    snapshot.qpos.assign(mData->qpos, mData->qpos + mModel->nq);
    snapshot.qvel.assign(mData->qvel, mData->qvel + mModel->nv);
    snapshot.ctrl.assign(mData->ctrl, mData->ctrl + mModel->nu);

    int siteId = mj_name2id(mModel, mjOBJ_SITE, "tcp");
    if (siteId < 0)
    {
        siteId = mj_name2id(mModel, mjOBJ_SITE, "pinch");
    }
    if (siteId >= 0)
    {
        const double* pos = mData->site_xpos + 3 * siteId;
        const double* mat = mData->site_xmat + 9 * siteId;
        snapshot.eePos = { pos[0], pos[1], pos[2] };
        mju_mat2Quat(snapshot.eeQuat.data(), mat);
    }

    std::lock_guard<std::mutex> lock(mStateMutex);
    mState = std::move(snapshot);
}

std::vector<double> dxMuJoCoRobotSimulator::extractJointPositions() const
{
    std::vector<double> joints;
    if (!mModel || !mData)
    {
        return joints;
    }

    for (int jid = 0; jid < mModel->njnt; ++jid)
    {
        const int type = mModel->jnt_type[jid];
        if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
        {
            continue;
        }
        const int qposAdr = mModel->jnt_qposadr[jid];
        if (qposAdr >= 0 && qposAdr < mModel->nq)
        {
            joints.push_back(mData->qpos[qposAdr]);
        }
    }

    return joints;
}

void dxMuJoCoRobotSimulator::shutdown()
{
    if (mData)
    {
        mj_deleteData(mData);
        mData = nullptr;
    }
    if (mModel)
    {
        mj_deleteModel(mModel);
        mModel = nullptr;
    }
}

bool dxMuJoCoRobotSimulator::applyPoseByName(const char* poseName)
{
    if (!mModel || !mData || !poseName)
    {
        return false;
    }
    const int poseId = mj_name2id(mModel, mjOBJ_KEY, poseName);
    if (poseId < 0)
    {
        return false;
    }
    mj_resetDataKeyframe(mModel, mData, poseId);
    mPoseApplied = true;
    mPoseAppliedName = poseName;
    printPoseSummary(poseName);
    return true;
}

void dxMuJoCoRobotSimulator::printPoseSummary(const char* poseName) const
{
    if (!mModel || !mData)
    {
        return;
    }
    const char* jointNames[6] =
    {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };

    std::printf("  Pose       : %s\n", poseName ? poseName : "(unnamed)");
    std::printf("  curr_q_pos (deg) = (");
    bool first = true;
    for (int i = 0; i < 6; ++i)
    {
        const int jid = mj_name2id(mModel, mjOBJ_JOINT, jointNames[i]);
        if (jid < 0)
        {
            continue;
        }
        const int qposAdr = mModel->jnt_qposadr[jid];
        if (qposAdr < 0 || qposAdr >= mModel->nq)
        {
            continue;
        }
        const double deg = mData->qpos[qposAdr] * 180.0 / 3.141592653589793;
        if (!first)
        {
            std::printf(", ");
        }
        std::printf("%.2f", deg);
        first = false;
    }
    std::printf(")\n");
}

void dxMuJoCoRobotSimulator::printDetails() const
{
    if (!mModel || !mData)
    {
        std::printf("dxMuJoCoRobotSimulator: no model/data loaded.\n");
        return;
    }

    const std::string modelName = mModelPath.toStdString();

    int robotDof = 0;
    int gripperDof = 0;
    const char* robotJoints[6] =
    {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };
    const char* gripperJoints[4] =
    {
        "hande_left_finger_joint",
        "hande_right_finger_joint",
        "left_finger_slide",
        "right_finger_slide"
    };

    for (int i = 0; i < 6; ++i)
        if (mj_name2id(mModel, mjOBJ_JOINT, robotJoints[i]) >= 0)
        {
            ++robotDof;
        }
    for (int i = 0; i < 4; ++i)
        if (mj_name2id(mModel, mjOBJ_JOINT, gripperJoints[i]) >= 0)
        {
            ++gripperDof;
        }

    const char* toolName = "none";
    if (mj_name2id(mModel, mjOBJ_BODY, "hande") >= 0)
    {
        toolName = "Robotiq Hand-E";
    }
    else if (mj_name2id(mModel, mjOBJ_BODY, "gripper_base") >= 0)
    {
        toolName = "Generic gripper";
    }

    std::printf("=== Simulator Info ===\n");
    std::printf("  Robot      : %s\n", modelName.c_str());
    std::printf("  Tool       : %s\n", toolName);
    std::printf("  DoF\n");
    std::printf("    Robot    : %d\n", robotDof);
    std::printf("    Gripper  : %d\n", gripperDof);
    std::printf("  Totals\n");
    std::printf("    Joints   : %d\n", mModel->njnt);
    std::printf("    Cameras  : %d", mModel->ncam);
    if (mModel->ncam > 0)
    {
        std::printf(" (");
        for (int i = 0; i < mModel->ncam; ++i)
        {
            const char* name = mj_id2name(mModel, mjOBJ_CAMERA, i);
            if (i > 0)
            {
                std::printf(", ");
            }
            std::printf("%s", name ? name : "unnamed");
        }
        std::printf(")");
    }
    std::printf("\n");
    if (mPoseApplied)
    {
        std::printf("  Pose       : %s\n", mPoseAppliedName.c_str());
    }
    else
    {
        std::printf("  Pose       : none\n");
    }
}
