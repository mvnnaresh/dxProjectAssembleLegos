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

    int numJoints() const;
    int numActuators() const;

    std::vector<double> getQpos() const;
    std::vector<double> getQvel() const;
    std::vector<double> getCtrl() const;

    bool setCtrl(int actuatorIndex, double value);
    bool setCtrlByName(const std::string& actuatorName, double value);

    dxRobotViewerFactory* viewer() const { return m_viewer.get(); }

    mjModel* model() const { return m_model; }
    mjData* data() const { return m_data; }

private:
    void shutdown();

    std::string m_modelPath;
    bool m_createViewer = true;
    std::unique_ptr<dxRobotViewerFactory> m_viewer;
    mjModel* m_model = nullptr;
    mjData* m_data = nullptr;
};
