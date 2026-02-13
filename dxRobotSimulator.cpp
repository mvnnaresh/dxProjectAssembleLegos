#include "dxRobotSimulator.h"

#include <cmath>
#include <cstdio>

static bool set_joint_qpos(mjModel* model, mjData* data, const char* jointName, double value)
{
    if (!model || !data || !jointName) return false;
    const int jid = mj_name2id(model, mjOBJ_JOINT, jointName);
    if (jid < 0) return false;
    const int qpos_adr = model->jnt_qposadr[jid];
    if (qpos_adr < 0 || qpos_adr >= model->nq) return false;
    data->qpos[qpos_adr] = value;
    return true;
}

static bool set_actuator_ctrl(mjModel* model, mjData* data, const char* actuatorName, double value)
{
    if (!model || !data || !actuatorName) return false;
    const int aid = mj_name2id(model, mjOBJ_ACTUATOR, actuatorName);
    if (aid < 0) return false;
    data->ctrl[aid] = value;
    return true;
}

static void zero_qvel(mjModel* model, mjData* data)
{
    if (!model || !data) return;
    for (int i = 0; i < model->nv; ++i)
        data->qvel[i] = 0.0;
}

dxRobotSimulator::dxRobotSimulator(const std::string& modelPath, bool createViewer)
    : m_modelPath(modelPath), m_createViewer(createViewer)
{
}

dxRobotSimulator::~dxRobotSimulator()
{
    shutdown();
}

bool dxRobotSimulator::loadModel()
{
    shutdown();

    char err[1024] = { 0 };
    m_model = mj_loadXML(m_modelPath.c_str(), nullptr, err, sizeof(err));
    if (!m_model)
    {
        std::fprintf(stderr, "dxRobotSimulator: mj_loadXML failed for '%s'\nError: %s\n",
                     m_modelPath.c_str(), err);
        return false;
    }

    m_data = mj_makeData(m_model);
    if (!m_data)
    {
        std::fprintf(stderr, "dxRobotSimulator: mj_makeData failed.\n");
        shutdown();
        return false;
    }

    mj_forward(m_model, m_data);
    return true;
}

bool dxRobotSimulator::init()
{
    if (!loadModel()) return false;

    // Default home pose: UR10e 90-90 configuration, gripper open.
    set_joint_qpos(m_model, m_data, "shoulder_pan_joint", 0.0);
    set_joint_qpos(m_model, m_data, "shoulder_lift_joint", -1.5708);
    set_joint_qpos(m_model, m_data, "elbow_joint", 1.5708);
    set_joint_qpos(m_model, m_data, "wrist_1_joint", -1.5708);
    set_joint_qpos(m_model, m_data, "wrist_2_joint", -1.5708);
    set_joint_qpos(m_model, m_data, "wrist_3_joint", 0.0);
    set_actuator_ctrl(m_model, m_data, "fingers_actuator", 0.0);
    set_actuator_ctrl(m_model, m_data, "shoulder_pan", 0.0);
    set_actuator_ctrl(m_model, m_data, "shoulder_lift", -1.5708);
    set_actuator_ctrl(m_model, m_data, "elbow", 1.5708);
    set_actuator_ctrl(m_model, m_data, "wrist_1", -1.5708);
    set_actuator_ctrl(m_model, m_data, "wrist_2", -1.5708);
    set_actuator_ctrl(m_model, m_data, "wrist_3", 0.0);
    zero_qvel(m_model, m_data);
    mj_forward(m_model, m_data);

    if (m_createViewer)
    {
        m_viewer = std::make_unique<dxRobotViewerFactory>(m_model, m_data);
        if (!m_viewer->init())
        {
            m_viewer.reset();
            return false;
        }
    }
    return true;
}

void dxRobotSimulator::run()
{
    if (!m_model || !m_data)
    {
        if (!init()) return;
    }
    if (!m_viewer) return;

    while (!m_viewer->shouldClose())
    {
        step(1);
        m_viewer->renderOnce();
    }
}

void dxRobotSimulator::reset()
{
    if (!m_model || !m_data) return;
    mj_resetData(m_model, m_data);
    mj_forward(m_model, m_data);
}

void dxRobotSimulator::step(int steps)
{
    if (!m_model || !m_data) return;
    if (steps < 1) steps = 1;
    for (int i = 0; i < steps; ++i)
        mj_step(m_model, m_data);
}

int dxRobotSimulator::numJoints() const
{
    return m_model ? m_model->njnt : 0;
}

int dxRobotSimulator::numActuators() const
{
    return m_model ? m_model->nu : 0;
}

std::vector<double> dxRobotSimulator::getQpos() const
{
    std::vector<double> out;
    if (!m_model || !m_data) return out;
    out.assign(m_data->qpos, m_data->qpos + m_model->nq);
    return out;
}

std::vector<double> dxRobotSimulator::getQvel() const
{
    std::vector<double> out;
    if (!m_model || !m_data) return out;
    out.assign(m_data->qvel, m_data->qvel + m_model->nv);
    return out;
}

std::vector<double> dxRobotSimulator::getCtrl() const
{
    std::vector<double> out;
    if (!m_model || !m_data) return out;
    out.assign(m_data->ctrl, m_data->ctrl + m_model->nu);
    return out;
}

bool dxRobotSimulator::setCtrl(int actuatorIndex, double value)
{
    if (!m_model || !m_data) return false;
    if (actuatorIndex < 0 || actuatorIndex >= m_model->nu) return false;
    m_data->ctrl[actuatorIndex] = value;
    return true;
}

bool dxRobotSimulator::setCtrlByName(const std::string& actuatorName, double value)
{
    if (!m_model || !m_data) return false;
    const int id = mj_name2id(m_model, mjOBJ_ACTUATOR, actuatorName.c_str());
    if (id < 0) return false;
    m_data->ctrl[id] = value;
    return true;
}

void dxRobotSimulator::shutdown()
{
    if (m_viewer)
    {
        m_viewer.reset();
    }

    if (m_data)
    {
        mj_deleteData(m_data);
        m_data = nullptr;
    }
    if (m_model)
    {
        mj_deleteModel(m_model);
        m_model = nullptr;
    }
}
