#pragma once

#include <QObject>
#include <QTimer>
#include <QString>
#include <QMetaType>

#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

#include "dxMuJoCoRobotState.h"

class dxMuJoCoRobotSimulator : public QObject
{
    Q_OBJECT

public:
    explicit dxMuJoCoRobotSimulator(QObject* parent = nullptr);
    ~dxMuJoCoRobotSimulator() override;

    dxMuJoCoRobotSimulator(const dxMuJoCoRobotSimulator&) = delete;
    dxMuJoCoRobotSimulator& operator=(const dxMuJoCoRobotSimulator&) = delete;

    void setControlRateHz(double rateHz);
    double controlRateHz() const;

    mjModel* model() const;
    mjData* data() const;

    dxMuJoCoRobotState getRobotState() const;

    void setCtrlByName(const std::string& actuatorName, double value);

public slots:
    void loadModel(const QString& modelPath);
    void reset();
    void start();
    void stop();
    void setCtrlTargets(const std::vector<double>& targets);
    void setCtrlTargetsFromJointPositions(const std::vector<double>& jointPositions);
    void setJointPositions(const std::vector<double>& jointPositions);

signals:
    void modelLoaded(mjModel* model);
    void stateUpdated();
    void resetDone();
    void error(const QString& message);

private slots:
    void stepLoop();

private:
    enum class HoldMode
    {
        None,
        HoldJointTargets,
        HoldCtrlTargets
    };

    bool applyPoseByName(const char* poseName);
    void printPoseSummary(const char* poseName) const;
    void printDetails() const;
    void shutdown();
    void applyPendingTargets();
    bool hasPendingTargets() const;
    void applyCtrlTargetsDirect(const std::vector<double>& targets);
    void applyCtrlTargetsFromJointPositionsDirect(const std::vector<double>& jointPositions);
    void applyJointPositionsDirect(const std::vector<double>& jointPositions);
    void updateStateSnapshot();
    std::vector<double> extractJointPositions() const;

    mutable std::mutex mStateMutex;
    dxMuJoCoRobotState mState;

    mutable std::mutex mTargetMutex;
    std::vector<double> mPendingCtrlTargets;
    std::vector<double> mPendingQpos;
    std::vector<std::pair<std::string, double>> mPendingNamedCtrls;
    bool mHasCtrlTargets = false;
    bool mHasPendingQpos = false;
    HoldMode mHoldMode = HoldMode::HoldJointTargets;
    std::vector<double> mHoldJointTargets;
    std::vector<double> mHoldCtrlTargets;

    QString mModelPath;
    bool mPoseApplied = false;
    std::string mPoseAppliedName;

    mjModel* mModel = nullptr;
    mjData* mData = nullptr;

    QTimer* mTimer = nullptr;
    double mControlRateHz = 250.0;
    bool mRunning = false;
};

Q_DECLARE_METATYPE(std::vector<double>)
