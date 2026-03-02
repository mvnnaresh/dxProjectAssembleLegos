// MuJoCo simulator wrapper: load, drive control loop, and emit state updates.
#include "dxMuJoCoRobotSimulator.h"

#include <algorithm>
#include <cstring>
#include <cmath>

#include <QDebug>
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

std::vector<double> dxMuJoCoRobotSimulator::getBodyPoseByName(const std::string& name) const
{
    std::vector<double> out;
    if (!mModel || !mData || name.empty())
    {
        return out;
    }

    const int id = mj_name2id(mModel, mjOBJ_BODY, name.c_str());
    if (id < 0 || id >= mModel->nbody)
    {
        return out;
    }

    const double* pos = mData->xpos + 3 * id;
    const double* mat = mData->xmat + 9 * id;
    double quat[4] = { 1.0, 0.0, 0.0, 0.0 };
    mju_mat2Quat(quat, mat);

    out.reserve(7);
    out.push_back(pos[0]);
    out.push_back(pos[1]);
    out.push_back(pos[2]);
    out.push_back(quat[0]);
    out.push_back(quat[1]);
    out.push_back(quat[2]);
    out.push_back(quat[3]);

    return out;
}

std::vector<double> dxMuJoCoRobotSimulator::getGeomPoseByName(const std::string& name) const
{
    std::vector<double> out;
    if (!mModel || !mData || name.empty())
    {
        return out;
    }

    const int id = mj_name2id(mModel, mjOBJ_GEOM, name.c_str());
    if (id < 0 || id >= mModel->ngeom)
    {
        return out;
    }

    const double* pos = mData->geom_xpos + 3 * id;
    const double* mat = mData->geom_xmat + 9 * id;
    double quat[4] = { 1.0, 0.0, 0.0, 0.0 };
    mju_mat2Quat(quat, mat);

    out.reserve(7);
    out.push_back(pos[0]);
    out.push_back(pos[1]);
    out.push_back(pos[2]);
    out.push_back(quat[0]);
    out.push_back(quat[1]);
    out.push_back(quat[2]);
    out.push_back(quat[3]);
    return out;
}

void dxMuJoCoRobotSimulator::setCtrlTargets(const std::vector<double>& targets)
{
    std::lock_guard<std::mutex> lock(mTargetMutex);
    mPendingCtrlTargets = targets;
    mHasCtrlTargets = true;
    mHoldCtrlTargets = targets;
    if (mHoldMode != HoldMode::HoldJointTargets || mHoldJointTargets.empty())
    {
        mHoldMode = HoldMode::HoldCtrlTargets;
    }
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

void dxMuJoCoRobotSimulator::setArmDofCount(int armDofCount)
{
    mArmDofCount = armDofCount;
}

void dxMuJoCoRobotSimulator::setCtrlTargetsFromJointPositions(const std::vector<double>& jointPositions)
{
    if (!mModel || jointPositions.empty())
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

    std::vector<double> targets;
    targets.assign(mData->ctrl, mData->ctrl + mModel->nu);
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
        if (mArmDofCount >= 0 && idx >= mArmDofCount)
        {
            continue;
        }
        targets[static_cast<size_t>(aid)] = jointPositions[static_cast<size_t>(idx)];
    }

    std::lock_guard<std::mutex> lock(mTargetMutex);
    mPendingCtrlTargets = targets;
    mHasCtrlTargets = true;
    mHoldCtrlTargets = targets;
    mHoldMode = HoldMode::HoldCtrlTargets;
}

void dxMuJoCoRobotSimulator::setCtrlTargetsFromFullJointPositions(const std::vector<double>& jointPositions)
{
    if (!mModel || jointPositions.empty())
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

    std::vector<double> targets;
    targets.assign(mData->ctrl, mData->ctrl + mModel->nu);
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

        double value = jointPositions[static_cast<size_t>(idx)];
        if (mModel->jnt_limited[jid])
        {
            const double lo = mModel->jnt_range[2 * jid];
            const double hi = mModel->jnt_range[2 * jid + 1];
            if (value < lo)
            {
                value = lo;
            }
            else if (value > hi)
            {
                value = hi;
            }
        }
        targets[static_cast<size_t>(aid)] = value;
    }

    std::lock_guard<std::mutex> lock(mTargetMutex);
    mPendingCtrlTargets = targets;
    mHasCtrlTargets = true;
    mHoldCtrlTargets = targets;
    mHoldMode = HoldMode::HoldCtrlTargets;
}

void dxMuJoCoRobotSimulator::setToolRatio(double ratio)
{
    if (!mModel || !mData)
    {
        return;
    }
    if (ratio < 0.0)
    {
        ratio = 0.0;
    }
    else if (ratio > 1.0)
    {
        ratio = 1.0;
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

    const int totalDof = orderIndex;
    const int toolStart = (mArmDofCount >= 0) ? std::min(mArmDofCount, totalDof) : totalDof;

    std::vector<double> targets;
    targets.assign(mData->ctrl, mData->ctrl + mModel->nu);
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
        if (idx < 0 || idx < toolStart)
        {
            continue;
        }

        double value = 0.0;
        if (mModel->jnt_limited[jid])
        {
            const double lo = mModel->jnt_range[2 * jid];
            const double hi = mModel->jnt_range[2 * jid + 1];
            value = lo + ratio * (hi - lo);
        }
        targets[static_cast<size_t>(aid)] = value;
    }

    std::lock_guard<std::mutex> lock(mTargetMutex);
    mPendingCtrlTargets = targets;
    mHasCtrlTargets = true;
    mHoldCtrlTargets = targets;
    mHoldMode = HoldMode::HoldCtrlTargets;
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

void dxMuJoCoRobotSimulator::closeGripper()
{
    setGripperPosition(1.0);
}

void dxMuJoCoRobotSimulator::setGripperPosition(double ratio)
{
    if (!mModel || !mData || mModel->nu <= 0)
    {
        return;
    }
    std::vector<int> gripperActuators = getTendonActuatorIndices();
    if (gripperActuators.empty())
    {
        gripperActuators = getGripperJointActuatorIndices();
    }
    if (gripperActuators.empty())
    {
        return;
    }

    if (ratio < 0.0)
    {
        ratio = 0.0;
    }
    else if (ratio > 1.0)
    {
        ratio = 1.0;
    }
    std::vector<double> targets;
    targets.assign(mData->ctrl, mData->ctrl + mModel->nu);

    for (int aid : gripperActuators)
    {
        if (aid < 0 || aid >= mModel->nu)
        {
            continue;
        }
        double target = ratio;
        if (mModel->actuator_ctrllimited[aid])
        {
            const double lo = mModel->actuator_ctrlrange[2 * aid];
            const double hi = mModel->actuator_ctrlrange[2 * aid + 1];
            target = lo + ratio * (hi - lo);
        }
        targets[static_cast<size_t>(aid)] = target;
    }

    std::lock_guard<std::mutex> lock(mTargetMutex);
    if (mHoldMode == HoldMode::HoldJointTargets && !mHoldJointTargets.empty())
    {
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

        for (int aid : gripperActuators)
        {
            if (aid < 0 || aid >= mModel->nu)
            {
                continue;
            }
            const char* name = mj_id2name(mModel, mjOBJ_ACTUATOR, aid);
            if (name)
            {
                mPendingNamedCtrls.emplace_back(name, targets[static_cast<size_t>(aid)]);
            }

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
            if (idx >= 0 && idx < static_cast<int>(mHoldJointTargets.size()))
            {
                mHoldJointTargets[static_cast<size_t>(idx)] = targets[static_cast<size_t>(aid)];
            }
        }
        return;
    }

    mPendingCtrlTargets = targets;
    mHasCtrlTargets = true;
    mHoldCtrlTargets = targets;
    mHoldMode = HoldMode::HoldCtrlTargets;
}

void dxMuJoCoRobotSimulator::printContacts(int maxContacts, double minDist)
{
    (void)maxContacts;
    (void)minDist;
}

void dxMuJoCoRobotSimulator::printContactsForGeom(const QString& geomName, double minDist)
{
    (void)geomName;
    (void)minDist;
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
    qDebug() << "dxMuJoCoRobotSimulator loading model:" << mModelPath;

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

void dxMuJoCoRobotSimulator::shutdownSimulator()
{
    stop();
    shutdown();
}

void dxMuJoCoRobotSimulator::stepLoop()
{
    if (!mModel || !mData)
    {
        return;
    }

    bool hadPending = false;
    bool namedOnly = false;
    {
        std::lock_guard<std::mutex> lock(mTargetMutex);
        const bool hasNamed = !mPendingNamedCtrls.empty();
        hadPending = mHasCtrlTargets || mHasPendingQpos || hasNamed;
        namedOnly = hasNamed && !mHasCtrlTargets && !mHasPendingQpos;
    }
    applyPendingTargets();

    if (!hadPending || namedOnly)
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
                applyCtrlTargetsFromJointPositionsDirect(holdJoints);
            }
            else
            {
                applyJointPositionsDirect(holdJoints);
            }
        }
    }

    //!--- patch to fix dropping behaviour
    // Make sure bias forces are updated
    mj_forward(mModel, mData);

    // Gravity compensation for hinge/slide joints
    for (int jid = 0; jid < mModel->njnt; ++jid)
    {
        int type = mModel->jnt_type[jid];
        if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
            continue;

        int dofadr = mModel->jnt_dofadr[jid];

        if (dofadr >= 0 && dofadr < mModel->nv)
        {
            mData->qfrc_applied[dofadr] = mData->qfrc_bias[dofadr];
        }
    }
    //!--- end of patch

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
        if (mArmDofCount >= 0 && idx >= mArmDofCount)
        {
            continue;
        }
        mData->ctrl[aid] = jointPositions[static_cast<size_t>(idx)];
    }
}

void dxMuJoCoRobotSimulator::applyJointPositionsDirect(const std::vector<double>& jointPositions, bool onlyActuated)
{
    if (!mModel || !mData || jointPositions.empty())
    {
        return;
    }

    std::vector<char> actuated;
    if (onlyActuated)
    {
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

        actuated.assign(static_cast<size_t>(orderIndex), 0);
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
            if (idx >= 0 && idx < static_cast<int>(actuated.size()))
            {
                actuated[static_cast<size_t>(idx)] = 1;
            }
        }
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
            if (!onlyActuated ||
                    (jointIndex >= 0 && jointIndex < static_cast<int>(actuated.size()) &&
                     actuated[static_cast<size_t>(jointIndex)]))
            {
                mData->qpos[qposAdr] = jointPositions[static_cast<size_t>(jointIndex)];
            }
        }
        ++jointIndex;
    }
    mj_forward(mModel, mData);
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

std::vector<int> dxMuJoCoRobotSimulator::getGripperJointActuatorIndices() const
{
    std::vector<int> indices;
    if (!mModel)
    {
        return indices;
    }

    const char* jointNames[] = { "hande_left_finger_joint", "hande_right_finger_joint" };
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
        const char* jointName = mj_id2name(mModel, mjOBJ_JOINT, jid);
        if (!jointName)
        {
            continue;
        }
        for (const char* target : jointNames)
        {
            if (std::strcmp(jointName, target) == 0)
            {
                indices.push_back(aid);
                break;
            }
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

    snapshot.jointConf = extractJointPositions();
    snapshot.jointVel = extractJointVelocities();
    snapshot.actuatorInput.assign(mData->ctrl, mData->ctrl + mModel->nu);
    snapshot.worldQpos.assign(mData->qpos, mData->qpos + mModel->nq);

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

std::vector<double> dxMuJoCoRobotSimulator::extractJointVelocities() const
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
        const int dofAdr = mModel->jnt_dofadr[jid];
        if (dofAdr >= 0 && dofAdr < mModel->nv)
        {
            joints.push_back(mData->qvel[dofAdr]);
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
    (void)poseName;
}

void dxMuJoCoRobotSimulator::printDetails() const
{
}
