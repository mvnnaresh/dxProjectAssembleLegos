#pragma once

#include <array>
#include <string>
#include <vector>
#include <utility>

#include <QObject>
#include <QTimer>

#include "dxKinMuJoCo.h"
#include "dxMuJoCoRobotSimulator.h"

class demo : public QObject
{
    Q_OBJECT

public:
    enum class EndBehavior
    {
        StopCommands,
        KeepLastPose
    };
    explicit demo(dxMuJoCoRobotSimulator* simulator, QObject* parent = nullptr);

    bool init();

    void testKinematics();

    void testPlannerSimple();

    void testPlannerCartesian();
    void testPickAndPlace();
    void testCamera();

    void closeGripper();
    void openGripper();
    void setGripperPosition(double ratio);

    void setEndBehavior(EndBehavior behavior);

signals:
    void ctrlTargetsReady(const std::vector<double>& targets);

    void updateJointConfig(const std::vector<double>& joints);

    void ctrlTargetsFromJointsReady(const std::vector<double>& joints);

    void drawTrajectory(const std::vector<std::array<double, 3>>& points);
    void drawFrames(const std::vector<std::array<double, 12>>& frames);
    void closeGripperRequested();
    void gripperPositionRequested(double ratio);
    void trajectoryFinished();

    void updateUIMessage(std::string msg);
    void cameraStreamRequested(bool enabled);

private:
    void startTrajectoryPlayback();
    //void buildDofGroups();
    //std::vector<double> extractArmDof(const std::vector<double>& dofQpos) const;
    //std::vector<double> expandArmDof(const std::vector<double>& baseDof,
    //                                 const std::vector<double>& armDof) const;
    bool planCartesianTo(const std::vector<double>& startPose,
                         const std::vector<double>& goalPose,
                         std::vector<std::vector<double>>& trajectory,
                         int steps);
    bool buildPoseFromJoints(const std::vector<double>& joints, std::vector<double>& outPose);
    bool sendRobotTo(const std::vector<double>& fromPose,
                     const std::vector<double>& toPose,
                     int steps);
    void waitSteps(int holdSteps);
    void setGripperHoldRatio(double ratio);

    dxMuJoCoRobotSimulator* mSim = nullptr;

    std::vector<std::vector<double>> mTrajectory;
    std::vector<std::pair<size_t, double>> mGripperEvents;
    size_t mGripperEventIndex = 0;
    bool mGripperLatched = false;
    double mGripperLatchedValue = 0.0;

    size_t mTrajectoryIndex = 0;
    QTimer* mTrajectoryTimer = nullptr;
    EndBehavior mEndBehavior = EndBehavior::StopCommands;
    std::vector<int> mArmDofIndices;
    std::vector<int> mGripperDofIndices;

    std::unique_ptr<dxKinMuJoCo> mKin;

    bool mGripperHoldEnabled = false;
    double mGripperHoldRatio = 0.0;

};
