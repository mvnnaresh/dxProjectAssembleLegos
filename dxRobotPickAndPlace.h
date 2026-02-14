#pragma once

class dxRobotSimulator;

class dxRobotPickAndPlace
{
public:
    explicit dxRobotPickAndPlace(dxRobotSimulator* simulator);

    void reset();
    void update();

private:
    dxRobotSimulator* m_sim = nullptr;
};
