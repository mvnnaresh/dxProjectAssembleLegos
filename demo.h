#pragma once

#include <array>
#include <string>
#include <vector>
#include <utility>

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
    bool planCartesianTo(const std::vector<double>& startPose,
                         const std::vector<double>& goalPose,
                         std::vector<std::vector<double>>& trajectory,
                         int steps);
    bool sendRobotTo(const std::vector<double>& fromPose,
                     const std::vector<double>& toPose,
                     int steps);
    void waitSteps(int holdSteps);

    dxMujocoInterface* mInterface = nullptr;

    std::vector<std::vector<double>> mTrajectory;
    size_t mTrajectoryIndex = 0;
    QTimer* mTrajectoryTimer = nullptr;
    EndBehavior mEndBehavior = EndBehavior::StopCommands;

    std::unique_ptr<dxKinMuJoCo> mKin;
};
