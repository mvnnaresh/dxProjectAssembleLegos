#include "dxMujocoInterface.h"

#include <QMetaObject>
#include <QMetaType>

dxMujocoInterface::dxMujocoInterface(QObject* parent)
    : QObject(parent)
{
    qRegisterMetaType<std::vector<double>>("std::vector<double>");

    mViewer = new dxMuJoCoRobotViewer();
}

dxMujocoInterface::~dxMujocoInterface()
{
    shutdown();
}

dxMuJoCoRobotViewer* dxMujocoInterface::viewer() const
{
    return mViewer;
}

mjModel* dxMujocoInterface::model() const
{
    return mSim ? mSim->model() : nullptr;
}

mjData* dxMujocoInterface::data() const
{
    return mSim ? mSim->data() : nullptr;
}

dxMuJoCoRobotState dxMujocoInterface::getRobotState() const
{
    return mSim ? mSim->getRobotState() : dxMuJoCoRobotState();
}

std::vector<double> dxMujocoInterface::getBodyPoseByName(const std::string& name) const
{
    return mSim ? mSim->getBodyPoseByName(name) : std::vector<double>();
}

std::vector<double> dxMujocoInterface::getGeomPoseByName(const std::string& name) const
{
    return mSim ? mSim->getGeomPoseByName(name) : std::vector<double>();
}

void dxMujocoInterface::setControlRateHz(double rateHz)
{
    if (mSim)
    {
        QMetaObject::invokeMethod(mSim.get(), "setControlRateHz", Qt::QueuedConnection,
                                  Q_ARG(double, rateHz));
    }
}

void dxMujocoInterface::start()
{
    if (mSim)
    {
        QMetaObject::invokeMethod(mSim.get(), "start", Qt::QueuedConnection);
    }
}

void dxMujocoInterface::stop()
{
    if (mSim)
    {
        QMetaObject::invokeMethod(mSim.get(), "stop", Qt::QueuedConnection);
    }
}

void dxMujocoInterface::setCtrlTargets(const std::vector<double>& targets)
{
    if (mSim)
    {
        QMetaObject::invokeMethod(mSim.get(), "setCtrlTargets", Qt::QueuedConnection,
                                  Q_ARG(std::vector<double>, targets));
    }
}

void dxMujocoInterface::setCtrlTargetsFromJointPositions(const std::vector<double>& jointPositions)
{
    if (mSim)
    {
        QMetaObject::invokeMethod(mSim.get(), "setCtrlTargetsFromJointPositions", Qt::QueuedConnection,
                                  Q_ARG(std::vector<double>, jointPositions));
    }
}

void dxMujocoInterface::setCtrlTargetsFromFullJointPositions(const std::vector<double>& jointPositions)
{
    if (mSim)
    {
        QMetaObject::invokeMethod(mSim.get(), "setCtrlTargetsFromFullJointPositions", Qt::QueuedConnection,
                                  Q_ARG(std::vector<double>, jointPositions));
    }
}

void dxMujocoInterface::setJointPositions(const std::vector<double>& jointPositions)
{
    if (mSim)
    {
        QMetaObject::invokeMethod(mSim.get(), "setJointPositions", Qt::QueuedConnection,
                                  Q_ARG(std::vector<double>, jointPositions));
    }
}

void dxMujocoInterface::setArmDofCount(int armDofCount)
{
    if (mSim)
    {
        QMetaObject::invokeMethod(mSim.get(), "setArmDofCount", Qt::QueuedConnection,
                                  Q_ARG(int, armDofCount));
    }
}

void dxMujocoInterface::closeGripper()
{
    if (mSim)
    {
        QMetaObject::invokeMethod(mSim.get(), "closeGripper", Qt::QueuedConnection);
    }
}

void dxMujocoInterface::setGripperPosition(double ratio)
{
    if (mSim)
    {
        QMetaObject::invokeMethod(mSim.get(), "setGripperPosition", Qt::QueuedConnection,
                                  Q_ARG(double, ratio));
    }
}

void dxMujocoInterface::loadModel(const QString& modelPath)
{
    ensureSimulator();
    if (mSim)
    {
        QMetaObject::invokeMethod(mSim.get(), "loadModel", Qt::QueuedConnection,
                                  Q_ARG(QString, modelPath));
    }
}

void dxMujocoInterface::reset()
{
    if (mSim)
    {
        QMetaObject::invokeMethod(mSim.get(), "reset", Qt::QueuedConnection);
    }
}

void dxMujocoInterface::shutdownThread()
{
    if (mSimThread)
    {
        mSimThread->quit();
        mSimThread->wait();
        mSimThread = nullptr;
    }
}

void dxMujocoInterface::ensureSimulator()
{
    if (mSim)
    {
        return;
    }

    mSim = std::make_unique<dxMuJoCoRobotSimulator>();
    mSimThread = new QThread(this);
    mSim->moveToThread(mSimThread);

    connect(mSim.get(), &dxMuJoCoRobotSimulator::modelLoaded, this, [this](mjModel* model)
    {
        mModelLoaded = (model != nullptr);
        emit modelLoaded(model);
    });
    connect(mSim.get(), &dxMuJoCoRobotSimulator::stateUpdated, this, &dxMujocoInterface::stateUpdated);
    connect(mSim.get(), &dxMuJoCoRobotSimulator::resetDone, this, &dxMujocoInterface::resetDone);
    connect(mSim.get(), &dxMuJoCoRobotSimulator::error, this, &dxMujocoInterface::error);

    mSimThread->start();
}

void dxMujocoInterface::shutdown()
{
    if (mShutdown)
    {
        return;
    }
    mShutdown = true;

    if (mSim && mModelLoaded)
    {
        QMetaObject::invokeMethod(mSim.get(), "stop", Qt::BlockingQueuedConnection);
        QMetaObject::invokeMethod(mSim.get(), "shutdownSimulator", Qt::BlockingQueuedConnection);
    }
    if (mSim)
    {
        QMetaObject::invokeMethod(mSim.get(), "deleteLater", Qt::BlockingQueuedConnection);
        mSim.release();
    }
    shutdownThread();
    mModelLoaded = false;
}

void dxMujocoInterface::clearViewer()
{
    mViewer = nullptr;
}
