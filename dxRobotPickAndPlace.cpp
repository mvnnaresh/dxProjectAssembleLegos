#include "dxRobotPickAndPlace.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "dxRobotSimulator.h"

dxRobotPickAndPlace::dxRobotPickAndPlace(dxRobotSimulator* simulator)
    : m_sim(simulator)
{
    reset();
    initIds();
}

void dxRobotPickAndPlace::reset()
{
    m_phase = Phase::Home;
    m_lastPhase = m_phase;
    m_graspClosing = false;
    m_phaseStart = m_sim && m_sim->data() ? m_sim->data()->time : 0.0;
    m_currentTargets = m_home;
    setArmTargets(m_home);
    m_gripperCommand = m_open;
    applyGripperCommand();
    std::printf("PickAndPlace phase: %s\n", phaseName());
}

void dxRobotPickAndPlace::update()
{
    if (!m_sim || !m_sim->data())
        return;

    const double now = m_sim->data()->time;
    const double elapsed = now - m_phaseStart;
    const mjModel* model = m_sim->model();
    mjData* data = m_sim->data();
    if (!model || !data) return;

    double cubePos[3] = { 0.55, 0.0, 0.05 };
    if (m_cubeBodyId >= 0)
    {
        const double* pos = data->xpos + 3 * m_cubeBodyId;
        cubePos[0] = pos[0];
        cubePos[1] = pos[1];
        cubePos[2] = pos[2];
    }
    const double approachPos[3] = {
        cubePos[0] + m_targetOffset[0],
        cubePos[1] + m_targetOffset[1],
        cubePos[2] + 0.12 + m_targetOffset[2]
    };
    const double graspPos[3] = {
        cubePos[0] + m_targetOffset[0],
        cubePos[1] + m_targetOffset[1],
        cubePos[2] + 0.045 + m_targetOffset[2]
    };
    const double placePos[3] = {
        cubePos[0] + m_placeOffset[0],
        cubePos[1] + m_placeOffset[1],
        cubePos[2] + 0.08 + m_placeOffset[2]
    };

    if (m_phase != m_lastPhase)
    {
        m_lastPhase = m_phase;
        m_graspClosing = false;
        switch (m_phase)
        {
        case Phase::Home:
        case Phase::Approach:
        case Phase::Release:
        case Phase::Retreat:
            m_gripperCommand = m_open;
            break;
        case Phase::Lift:
        case Phase::MoveToPlace:
            m_gripperCommand = m_closed;
            break;
        case Phase::Grasp:
            m_gripperCommand = m_open;
            break;
        }
        applyGripperCommand();
    }

    switch (m_phase)
    {
    case Phase::Home:
        m_currentTargets = m_home;
        setArmTargets(m_home);
        if (elapsed > m_phaseDuration) advancePhase();
        break;
    case Phase::Approach:
    {
        std::array<double, 6> desired;
        if (computeIKStep(approachPos, desired))
            blendTargets(desired);
        if ((elapsed > m_phaseDuration && atTarget(approachPos, m_targetTolerance)) || elapsed > m_phaseTimeout)
        {
            advancePhase();
        }
        break;
    }
    case Phase::Grasp:
    {
        const bool onTarget = atTarget(graspPos, m_targetTolerance);
        if (onTarget)
        {
            m_graspClosing = true;
            std::array<double, 6> desired;
            if (computeIKStep(graspPos, desired))
                blendTargets(desired);
            if (m_graspClosing)
            {
                m_gripperCommand = m_closed;
                applyGripperCommand();
                if (gripperClosed())
                    advancePhase();
            }
        }
        else
        {
            std::array<double, 6> desired;
            if (computeIKStep(graspPos, desired))
                blendTargets(desired);
        }
        break;
    }
    case Phase::Lift:
    {
        std::array<double, 6> desired;
        if (computeIKStep(approachPos, desired))
            blendTargets(desired);
        if ((elapsed > m_phaseDuration && atTarget(approachPos, m_targetTolerance)) || elapsed > m_phaseTimeout)
            advancePhase();
        break;
    }
    case Phase::MoveToPlace:
    {
        std::array<double, 6> desired;
        if (computeIKStep(placePos, desired))
            blendTargets(desired);
        if ((elapsed > m_phaseDuration && atTarget(placePos, m_targetTolerance)) || elapsed > m_phaseTimeout)
            advancePhase();
        break;
    }
    case Phase::Release:
    {
        std::array<double, 6> desired;
        if (computeIKStep(placePos, desired))
            blendTargets(desired);
        if (elapsed > m_phaseDuration)
            advancePhase();
        break;
    }
    case Phase::Retreat:
    {
        std::array<double, 6> desired;
        if (computeIKStep(approachPos, desired))
            blendTargets(desired);
        if (elapsed > m_phaseDuration)
            advancePhase();
        break;
    }
    }
}

void dxRobotPickAndPlace::setArmTargets(const std::array<double, 6>& q)
{
    if (!m_sim) return;
    m_sim->setCtrlByName("shoulder_pan", q[0]);
    m_sim->setCtrlByName("shoulder_lift", q[1]);
    m_sim->setCtrlByName("elbow", q[2]);
    m_sim->setCtrlByName("wrist_1", q[3]);
    m_sim->setCtrlByName("wrist_2", q[4]);
    m_sim->setCtrlByName("wrist_3", q[5]);
}

void dxRobotPickAndPlace::setGripper(double value)
{
    if (!m_sim) return;
    if (!m_sim->data()) return;
    mjData* data = m_sim->data();
    const double dt = m_sim->model() ? m_sim->model()->opt.timestep : 0.002;
    const double maxStep = m_gripperRate * dt;
    const int aid = mj_name2id(m_sim->model(), mjOBJ_ACTUATOR, "fingers_actuator");
    if (aid < 0) return;
    const double current = data->ctrl[aid];
    const double minCtrl = std::min(m_open, m_closed);
    const double maxCtrl = std::max(m_open, m_closed);
    if (value < minCtrl) value = minCtrl;
    if (value > maxCtrl) value = maxCtrl;
    const double diff = value - current;
    double next = value;
    if (diff > maxStep)
        next = current + maxStep;
    else if (diff < -maxStep)
        next = current - maxStep;
    data->ctrl[aid] = next;
}

void dxRobotPickAndPlace::advancePhase()
{
    m_phaseStart = m_sim && m_sim->data() ? m_sim->data()->time : 0.0;

    switch (m_phase)
    {
    case Phase::Home:
        m_phase = Phase::Approach;
        break;
    case Phase::Approach:
        m_phase = Phase::Grasp;
        break;
    case Phase::Grasp:
        m_phase = Phase::Lift;
        break;
    case Phase::Lift:
        m_phase = Phase::MoveToPlace;
        break;
    case Phase::MoveToPlace:
        m_phase = Phase::Release;
        break;
    case Phase::Release:
        m_phase = Phase::Retreat;
        break;
    case Phase::Retreat:
        m_phase = Phase::Home;
        break;
    }
    std::printf("PickAndPlace phase: %s\n", phaseName());
}

const char* dxRobotPickAndPlace::phaseName() const
{
    switch (m_phase)
    {
    case Phase::Home: return "Home";
    case Phase::Approach: return "Approach";
    case Phase::Grasp: return "Grasp";
    case Phase::Lift: return "Lift";
    case Phase::MoveToPlace: return "MoveToPlace";
    case Phase::Release: return "Release";
    case Phase::Retreat: return "Retreat";
    }
    return "Unknown";
}

bool dxRobotPickAndPlace::initIds()
{
    if (!m_sim || !m_sim->model() || !m_sim->data())
        return false;

    mjModel* model = m_sim->model();

    m_siteId = mj_name2id(model, mjOBJ_SITE, "pinch");
    m_cubeBodyId = mj_name2id(model, mjOBJ_BODY, "cube");

    const char* jointNames[6] = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };

    m_jointIds.clear();
    m_qposAdr.clear();
    m_dofAdr.clear();
    for (int i = 0; i < 6; ++i)
    {
        const int jid = mj_name2id(model, mjOBJ_JOINT, jointNames[i]);
        if (jid < 0) return false;
        m_jointIds.push_back(jid);
        m_qposAdr.push_back(model->jnt_qposadr[jid]);
        m_dofAdr.push_back(model->jnt_dofadr[jid]);
    }

    if (m_sim->data())
    {
        for (int i = 0; i < 6; ++i)
            m_currentTargets[i] = m_sim->data()->qpos[m_qposAdr[i]];
    }

    return m_siteId >= 0;
}

bool dxRobotPickAndPlace::computeIKStep(const double target[3], std::array<double, 6>& desired)
{
    if (!m_sim || !m_sim->model() || !m_sim->data() || m_siteId < 0)
        return false;

    mjModel* model = m_sim->model();
    mjData* data = m_sim->data();

    mj_forward(model, data);

    const double* sitePos = data->site_xpos + 3 * m_siteId;
    double err[3] = {
        target[0] - sitePos[0],
        target[1] - sitePos[1],
        target[2] - sitePos[2]
    };

    std::vector<mjtNum> jacp(3 * model->nv, 0.0);
    std::vector<mjtNum> jacr(3 * model->nv, 0.0);
    mj_jacSite(model, data, jacp.data(), jacr.data(), m_siteId);

    double Jpos[3][6] = { 0 };
    double Jrot[3][6] = { 0 };
    for (int c = 0; c < 6; ++c)
    {
        const int dof = m_dofAdr[c];
        Jpos[0][c] = jacp[0 * model->nv + dof];
        Jpos[1][c] = jacp[1 * model->nv + dof];
        Jpos[2][c] = jacp[2 * model->nv + dof];
        Jrot[0][c] = jacr[0 * model->nv + dof];
        Jrot[1][c] = jacr[1 * model->nv + dof];
        Jrot[2][c] = jacr[2 * model->nv + dof];
    }

    const double* xmat = data->site_xmat + 9 * m_siteId;
    const double curZ[3] = { xmat[2], xmat[5], xmat[8] };
    const double desZ[3] = { 0.0, 0.0, -1.0 };
    const double rotErr[3] = {
        curZ[1] * desZ[2] - curZ[2] * desZ[1],
        curZ[2] * desZ[0] - curZ[0] * desZ[2],
        curZ[0] * desZ[1] - curZ[1] * desZ[0]
    };

    const double oriWeight = 0.4;
    double e6[6] = {
        err[0],
        err[1],
        err[2],
        oriWeight * rotErr[0],
        oriWeight * rotErr[1],
        oriWeight * rotErr[2]
    };

    double J[6][6] = { 0 };
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            J[r][c] = Jpos[r][c];
            J[r + 3][c] = oriWeight * Jrot[r][c];
        }
    }

    double A[6][6] = { 0 };
    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            double sum = 0.0;
            for (int k = 0; k < 6; ++k)
                sum += J[r][k] * J[c][k];
            A[r][c] = sum;
        }
        A[r][r] += m_damping * m_damping;
    }

    double y[6] = { 0 };
    if (!solveLinear6(A, e6, y))
        return false;

    double dq[6] = { 0 };
    for (int c = 0; c < 6; ++c)
        for (int r = 0; r < 6; ++r)
            dq[c] += J[r][c] * y[r];

    const double alpha = 0.2;
    for (int i = 0; i < 6; ++i)
    {
        const int qadr = m_qposAdr[i];
        desired[i] = data->qpos[qadr] + alpha * dq[i];
    }

    return true;
}

void dxRobotPickAndPlace::applyGripperCommand()
{
    setGripper(m_gripperCommand);
}

bool dxRobotPickAndPlace::gripperClosed() const
{
    if (!m_sim || !m_sim->model() || !m_sim->data())
        return false;
    const int aid = mj_name2id(m_sim->model(), mjOBJ_ACTUATOR, "fingers_actuator");
    if (aid < 0) return false;
    const double current = m_sim->data()->ctrl[aid];
    return std::abs(current - m_closed) < 0.002;
}

bool dxRobotPickAndPlace::solveLinear6(double A[6][6], const double b[6], double x[6]) const
{
    double aug[6][7] = { 0 };
    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
            aug[r][c] = A[r][c];
        aug[r][6] = b[r];
    }

    for (int i = 0; i < 6; ++i)
    {
        int piv = i;
        double maxv = std::abs(aug[i][i]);
        for (int r = i + 1; r < 6; ++r)
        {
            const double v = std::abs(aug[r][i]);
            if (v > maxv)
            {
                maxv = v;
                piv = r;
            }
        }
        if (maxv < 1e-12)
            return false;
        if (piv != i)
        {
            for (int c = i; c < 7; ++c)
                std::swap(aug[i][c], aug[piv][c]);
        }

        const double diag = aug[i][i];
        for (int c = i; c < 7; ++c)
            aug[i][c] /= diag;

        for (int r = 0; r < 6; ++r)
        {
            if (r == i) continue;
            const double f = aug[r][i];
            for (int c = i; c < 7; ++c)
                aug[r][c] -= f * aug[i][c];
        }
    }

    for (int i = 0; i < 6; ++i)
        x[i] = aug[i][6];
    return true;
}

void dxRobotPickAndPlace::blendTargets(const std::array<double, 6>& desired)
{
    for (int i = 0; i < 6; ++i)
    {
        const double diff = desired[i] - m_currentTargets[i];
        if (diff > m_maxJointStep)
            m_currentTargets[i] += m_maxJointStep;
        else if (diff < -m_maxJointStep)
            m_currentTargets[i] -= m_maxJointStep;
        else
            m_currentTargets[i] = desired[i];
    }

    setArmTargets(m_currentTargets);
}

bool dxRobotPickAndPlace::atTarget(const double target[3], double tol) const
{
    if (!m_sim || !m_sim->data() || m_siteId < 0)
        return false;

    const double* pos = m_sim->data()->site_xpos + 3 * m_siteId;
    const double dx = pos[0] - target[0];
    const double dy = pos[1] - target[1];
    const double dz = pos[2] - target[2];
    return (dx * dx + dy * dy + dz * dz) < (tol * tol);
}
