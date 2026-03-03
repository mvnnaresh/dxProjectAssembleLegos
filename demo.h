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
    void testNewPickAndPlace();
    void testLegoAssembly();
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
    struct BrickZInfo
    {
        double centerZ = 0.0;
        double topZ = 0.0;
        double bottomZ = 0.0;
        double height = 0.0;
        bool valid = false;
    };

    void startTrajectoryPlayback();
    bool planCartesianTo(const std::vector<double>& startPose,
                         const std::vector<double>& goalPose,
                         std::vector<std::vector<double>>& trajectory,
                         int steps);
    bool sendRobotTo(const std::vector<double>& fromPose,
                     const std::vector<double>& toPose,
                     int steps);
    bool sendRobotToBlocking(const std::vector<double>& fromPose,
                             const std::vector<double>& toPose,
                             int steps,
                             int sleepMs);
    bool sendRobotToBlockingFull(const std::vector<double>& fromPose,
                                 const std::vector<double>& toPose,
                                 int steps,
                                 int sleepMs);
    void waitSteps(int holdSteps);
    void waitStepsBlocking(int holdSteps, int sleepMs);
    void waitStepsBlockingFull(int holdSteps, int sleepMs);
    void setToolRatioFull(double ratio);
    bool runFullTrajectory(const std::vector<std::vector<double>>& armTrajectory);
    void waitFullSteps(int holdSteps);
    BrickZInfo computeBrickZInfo(const std::string& bodyName, const std::string& geomName) const;

    dxMujocoInterface* mInterface = nullptr;

    std::vector<std::vector<double>> mTrajectory;
    size_t mTrajectoryIndex = 0;
    QTimer* mTrajectoryTimer = nullptr;
    EndBehavior mEndBehavior = EndBehavior::StopCommands;
    bool mUseFullCtrl = false;
    int mArmDofCount = 6;
    std::vector<double> mFullCtrlTargets;
    double mToolRatio = 0.0;

    std::unique_ptr<dxKinMuJoCo> mKin;
};
