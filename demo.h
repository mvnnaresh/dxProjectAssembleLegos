#pragma once

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

signals:
    void ctrlTargetsReady(const std::vector<double>& targets);

    void jointPositionsReady(const std::vector<double>& joints);

    void ctrlTargetsFromJointsReady(const std::vector<double>& joints);

private:
    void startTrajectoryPlayback();

    dxMuJoCoRobotSimulator* mSim = nullptr;

    std::vector<std::vector<double>> mTrajectory;

    size_t mTrajectoryIndex = 0;
    QTimer* mTrajectoryTimer = nullptr;

    std::unique_ptr<dxKinMuJoCo> mKin;
};
