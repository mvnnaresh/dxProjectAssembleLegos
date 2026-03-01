#pragma once

#include <QObject>
#include <QThread>
#include <QString>

#include <memory>
#include <string>
#include <vector>

#include "dxMuJoCoRobotSimulator.h"
#include "dxMuJoCoRobotViewer.h"

class dxMujocoInterface : public QObject
{
    Q_OBJECT

public:
    explicit dxMujocoInterface(QObject* parent = nullptr);
    ~dxMujocoInterface() override;

    dxMujocoInterface(const dxMujocoInterface&) = delete;
    dxMujocoInterface& operator=(const dxMujocoInterface&) = delete;

    dxMuJoCoRobotViewer* viewer() const;
    mjModel* model() const;
    mjData* data() const;

    dxMuJoCoRobotState getRobotState() const;
    std::vector<double> getBodyPoseByName(const std::string& name) const;
    std::vector<double> getGeomPoseByName(const std::string& name) const;

    void setControlRateHz(double rateHz);
    void start();
    void stop();

    void setCtrlTargets(const std::vector<double>& targets);
    void setCtrlTargetsFromJointPositions(const std::vector<double>& jointPositions);
    void setJointPositions(const std::vector<double>& jointPositions);
    void closeGripper();
    void setGripperPosition(double ratio);

public slots:
    void loadModel(const QString& modelPath);
    void reset();

signals:
    void modelLoaded(mjModel* model);
    void stateUpdated();
    void resetDone();
    void error(const QString& message);

private:
    void shutdownThread();

    std::unique_ptr<dxMuJoCoRobotViewer> mViewer;
    std::unique_ptr<dxMuJoCoRobotSimulator> mSim;
    QThread* mSimThread = nullptr;
};
