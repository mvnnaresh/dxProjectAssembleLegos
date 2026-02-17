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
    explicit demo(dxMuJoCoRobotSimulator* simulator, QObject* parent = nullptr);

    bool init();

    void testKinematics();

    void testPlannerSimple();

    void testPlannerCartesian();

signals:
    void ctrlTargetsReady(const std::vector<double>& targets);

    void jointPositionsReady(const std::vector<double>& joints);

    void ctrlTargetsFromJointsReady(const std::vector<double>& joints);

    void drawTrajectory(const std::vector<std::array<double, 3>>& points);
    void closeGripperRequested();

private:
    void startTrajectoryPlayback();
    void applyGripperClose();
    void buildDofGroups();
    std::vector<double> extractArmDof(const std::vector<double>& dofQpos) const;
    std::vector<double> expandArmDof(const std::vector<double>& baseDof,
                                     const std::vector<double>& armDof) const;

    dxMuJoCoRobotSimulator* mSim = nullptr;

    std::vector<std::vector<double>> mTrajectory;

    size_t mTrajectoryIndex = 0;
    QTimer* mTrajectoryTimer = nullptr;
    std::vector<int> mArmDofIndices;
    std::vector<int> mGripperDofIndices;

    std::unique_ptr<dxKinMuJoCo> mKin;
};
