#pragma once

#include <array>
#include <string>
#include <vector>

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
    enum class PickPlaceStage
    {
        Idle,
        PreGraspMove,
        GraspMove,
        GripperCloseWait,
        MicroLiftMove,
        LiftMove
    };

    explicit demo(dxMuJoCoRobotSimulator* simulator, QObject* parent = nullptr);

    bool init();

    void testKinematics();

    void testPlannerSimple();

    void testPlannerCartesian();
    void testPickAndPlace();

    void closeGripper();
    void openGripper();
    void setGripperPosition(double ratio);
    void setCubeWeldActive(bool active);

    void setEndBehavior(EndBehavior behavior);

signals:
    void ctrlTargetsReady(const std::vector<double>& targets);

    void jointPositionsReady(const std::vector<double>& joints);

    void ctrlTargetsFromJointsReady(const std::vector<double>& joints);

    void drawTrajectory(const std::vector<std::array<double, 3>>& points);
    void drawFrames(const std::vector<std::array<double, 12>>& frames);
    void closeGripperRequested();
    void gripperPositionRequested(double ratio);
    void cubeWeldRequested(bool active);

private:
    void startTrajectoryPlayback();
    void applyGripperClose();
    void buildDofGroups();
    std::vector<double> extractArmDof(const std::vector<double>& dofQpos) const;
    std::vector<double> expandArmDof(const std::vector<double>& baseDof,
                                     const std::vector<double>& armDof) const;
    bool planCartesianTo(const std::vector<double>& startPose,
                         const std::vector<double>& goalPose,
                         std::vector<std::vector<double>>& trajectory,
                         int steps);
    bool buildPoseFromJoints(const std::vector<double>& joints, std::vector<double>& outPose);
    void advancePickAndPlace();

    dxMuJoCoRobotSimulator* mSim = nullptr;

    std::vector<std::vector<double>> mTrajectory;

    size_t mTrajectoryIndex = 0;
    QTimer* mTrajectoryTimer = nullptr;
    EndBehavior mEndBehavior = EndBehavior::StopCommands;
    PickPlaceStage mPickPlaceStage = PickPlaceStage::Idle;
    bool mPickPlaceActive = false;
    std::vector<double> mPickPlaceCubePose;
    std::vector<double> mPickPlacePreGraspPose;
    std::vector<double> mPickPlaceMicroLiftPose;
    int mPickPlaceWaitTicks = 0;
    int mPickPlaceWaitRemaining = 0;
    bool mPickPlaceContactLogPending = false;
    std::vector<int> mArmDofIndices;
    std::vector<int> mGripperDofIndices;

    std::unique_ptr<dxKinMuJoCo> mKin;
};
