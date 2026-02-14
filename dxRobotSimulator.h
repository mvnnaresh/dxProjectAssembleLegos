#pragma once

#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "dxRobotViewerFactory.h"

class dxRobotSimulator
{
public:
    explicit dxRobotSimulator(const std::string& modelPath, bool createViewer = true);
    ~dxRobotSimulator();

    dxRobotSimulator(const dxRobotSimulator&) = delete;
    dxRobotSimulator& operator=(const dxRobotSimulator&) = delete;

    bool loadModel();
    bool init();
    void run();
    void reset();
    void step(int steps = 1);
    void printDetails() const;
    void printCurrentPoseDegrees() const;

    int numJoints() const;
    int numActuators() const;

    std::vector<double> getQpos() const;
    std::vector<double> getQvel() const;
    std::vector<double> getCtrl() const;

    bool setCtrl(int actuatorIndex, double value);
    bool setCtrlByName(const std::string& actuatorName, double value);
    void setCtrlTargets(const std::vector<double>& targets);

    dxRobotViewerFactory* viewer() const { return mViewer.get(); }

    mjModel* model() const { return mModel; }
    mjData* data() const { return mData; }

private:
    bool applyPoseByName(const char* poseName);
    void printPoseSummary(const char* poseName) const;
    void shutdown();

    std::string mModelPath;
    bool mCreateViewer = true;
    bool mPoseApplied = false;
    std::string mPoseAppliedName;
    std::unique_ptr<dxRobotViewerFactory> mViewer;
    mjModel* mModel = nullptr;
    mjData* mData = nullptr;
};

