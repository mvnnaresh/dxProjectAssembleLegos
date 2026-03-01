#pragma once

#include <array>
#include <string>
#include <vector>
#include <utility>

#include <functional>

#include <QObject>
#include <QTimer>

#include "dxKinMuJoCo.h"
#include "dxMujocoInterface.h"

class demo : public QObject
{
    Q_OBJECT

public:
    enum class EndBehavior
    {
        StopCommands,
        KeepLastPose
    };
    explicit demo(dxMujocoInterface* interfacePtr, QObject* parent = nullptr);

    bool init();

    void testKinematics();

    void testPlannerSimple();

    void testPlannerCartesian();
    void testPickAndPlace();
    void testCamera();
    void testCamera3D();

    void closeGripper();
    void openGripper();
    void setGripperPosition(double ratio);

    void setEndBehavior(EndBehavior behavior);

signals:
    void drawTrajectory(const std::vector<std::array<double, 3>>& points);
    void drawFrames(const std::vector<std::array<double, 12>>& frames);
    void trajectoryFinished();

    void updateUIMessage(std::string msg);

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
    bool sendRobotTo(const std::vector<double>& fromPose,
                     const std::vector<double>& toPose,
                     int steps);
    void waitSteps(int holdSteps);
    void setGripperHoldRatio(double ratio);

    dxMujocoInterface* mInterface = nullptr;

    std::vector<std::vector<double>> mTrajectory;
    std::vector<std::pair<size_t, double>> mGripperEvents;
    size_t mGripperEventIndex = 0;
    bool mGripperLatched = false;
    double mGripperLatchedValue = 0.0;

    size_t mTrajectoryIndex = 0;
    QTimer* mTrajectoryTimer = nullptr;
    EndBehavior mEndBehavior = EndBehavior::StopCommands;

    bool mGripperHoldEnabled = false;
    double mGripperHoldRatio = 0.0;

    std::unique_ptr<dxKinMuJoCo> mKin;
};
