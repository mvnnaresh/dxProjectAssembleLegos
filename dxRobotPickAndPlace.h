#pragma once

#include <array>
#include <vector>

class dxRobotSimulator;

class dxRobotPickAndPlace
{
public:
    explicit dxRobotPickAndPlace(dxRobotSimulator* simulator);

    void reset();
    void update();

private:
    enum class Phase
    {
        Home = 0,
        Approach,
        Grasp,
        Lift,
        MoveToPlace,
        Release,
        Retreat
    };

    void setArmTargets(const std::array<double, 6>& q);
    void setGripper(double value);
    void advancePhase();
    bool initIds();
    bool computeIKStep(const double target[3], std::array<double, 6>& desired);
    bool gripperClosed() const;
    bool solveLinear6(double A[6][6], const double b[6], double x[6]) const;
    void blendTargets(const std::array<double, 6>& desired);
    bool atTarget(const double target[3], double tol) const;
    const char* phaseName() const;
    void applyGripperCommand();

    dxRobotSimulator* m_sim = nullptr;
    Phase m_phase = Phase::Home;
    Phase m_lastPhase = Phase::Home;
    double m_phaseStart = 0.0;
    bool m_graspClosing = false;

    std::array<double, 6> m_home = { 0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0 };
    std::array<double, 6> m_approach = { 0.0, -1.2, 1.2, -1.6, -1.57, 0.0 };
    std::array<double, 6> m_grasp = { 0.0, -1.3, 1.35, -1.65, -1.57, 0.0 };
    std::array<double, 6> m_lift = { 0.0, -1.1, 1.1, -1.4, -1.57, 0.0 };
    std::array<double, 6> m_place = { 0.6, -1.1, 1.1, -1.4, -1.57, 0.0 };
    std::array<double, 6> m_retreat = { 0.6, -1.2, 1.2, -1.6, -1.57, 0.0 };
    std::array<double, 6> m_currentTargets = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    double m_open = 0.04;
    double m_closed = 0.0;
    double m_gripperRate = 0.6;
    double m_gripperCommand = 0.03;

    double m_phaseDuration = 0.7;
    double m_phaseTimeout = 10.0;
    double m_targetTolerance = 0.005;
    double m_damping = 0.1;
    double m_maxJointStep = 0.02;

    std::array<double, 3> m_targetOffset = { 0.0, 0.0, 0.0 };
    std::array<double, 3> m_placeOffset = { 0.2, -0.2, 0.0 };

    int m_siteId = -1;
    int m_cubeBodyId = -1;
    std::vector<int> m_jointIds;
    std::vector<int> m_qposAdr;
    std::vector<int> m_dofAdr;
};
