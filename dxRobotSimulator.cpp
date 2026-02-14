// MuJoCo simulator wrapper for loading a model, driving a viewer, and stepping.
#include "dxRobotSimulator.h"

#include <cmath>
#include <cstdio>

// Helpers for manipulating MuJoCo state by name.
// Set a named joint position directly in qpos for deterministic initialization.
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

// Write a named actuator control value without applying any control logic.
static bool set_actuator_ctrl(mjModel* model, mjData* data, const char* actuatorName, double value)
{
    if (!model || !data || !actuatorName) return false;
    const int aid = mj_name2id(model, mjOBJ_ACTUATOR, actuatorName);
    if (aid < 0) return false;
    data->ctrl[aid] = value;
    return true;
}

// Clear generalized velocities to remove residual motion on startup/reset.
static void zero_qvel(mjModel* model, mjData* data)
{
    if (!model || !data) return;
    for (int i = 0; i < model->nv; ++i)
        data->qvel[i] = 0.0;
}

// Store config for model path and optional viewer creation.
dxRobotSimulator::dxRobotSimulator(const std::string& modelPath, bool createViewer)
    : mModelPath(modelPath), mCreateViewer(createViewer)
{
}

// Ensure MuJoCo data/model and viewer resources are released.
dxRobotSimulator::~dxRobotSimulator()
{
    shutdown();
}

// Load MuJoCo XML into a model and allocate its runtime data.
bool dxRobotSimulator::loadModel()
{
    shutdown();

    // Load the XML model and allocate data; report MuJoCo loader errors.
    char err[1024] = { 0 };
    mModel = mj_loadXML(mModelPath.c_str(), nullptr, err, sizeof(err));
    if (!mModel)
    {
        std::fprintf(stderr, "dxRobotSimulator: mj_loadXML failed for '%s'\nError: %s\n",
                     mModelPath.c_str(), err);
        return false;
    }

    mData = mj_makeData(mModel);
    if (!mData)
    {
        std::fprintf(stderr, "dxRobotSimulator: mj_makeData failed.\n");
        shutdown();
        return false;
    }

    // Ensure derived quantities (xpos, xmat, etc.) are consistent.
    mj_forward(mModel, mData);
    return true;
}

// Initialize the simulator state using a home keyframe if present, then optional viewer.
bool dxRobotSimulator::init()
{
    if (!loadModel()) return false;

    // Prefer a named HOME_POS pose when the model provides one (UR10e 90-90 start).
    mPoseApplied = false;
    mPoseAppliedName.clear();
    if (!applyPoseByName("HOME_POS"))
        applyPoseByName("home");

    // Avoid residual velocities before starting.
    zero_qvel(mModel, mData);
    mj_forward(mModel, mData);

    if (mCreateViewer)
    {
        // Viewer owns rendering state; keep it optional for headless runs.
        mViewer = std::make_unique<dxRobotViewerFactory>(mModel, mData);
        if (!mViewer->init())
        {
            mViewer.reset();
            return false;
        }
    }
    printDetails();
    return true;
}

// Run a minimal step/render loop until the viewer is closed.
void dxRobotSimulator::run()
{
    if (!mModel || !mData)
    {
        if (!init()) return;
    }
    if (!mViewer) return;

    // Simple real-time loop: step simulation then render.
    while (!mViewer->shouldClose())
    {
        step(1);
        mViewer->renderOnce();
    }
}

// Reset simulation state to model defaults and recompute derived state.
void dxRobotSimulator::reset()
{
    if (!mModel || !mData) return;
    // Reset to model defaults and recompute derived state.
    mj_resetData(mModel, mData);
    mj_forward(mModel, mData);
}

// Advance the simulation by a fixed number of MuJoCo steps.
void dxRobotSimulator::step(int steps)
{
    if (!mModel || !mData) return;
    if (steps < 1) steps = 1;
    for (int i = 0; i < steps; ++i)
        mj_step(mModel, mData);
}

// Print model and runtime details after initialization.
void dxRobotSimulator::printDetails() const
{
    if (!mModel || !mData)
    {
        std::printf("dxRobotSimulator: no model/data loaded.\n");
        return;
    }

    const char* modelName = mModelPath.c_str();

    int robotDof = 0;
    int gripperDof = 0;
    const char* robotJoints[6] = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };
    const char* gripperJoints[4] = {
        "hande_left_finger_joint",
        "hande_right_finger_joint",
        "left_finger_slide",
        "right_finger_slide"
    };

    for (int i = 0; i < 6; ++i)
        if (mj_name2id(mModel, mjOBJ_JOINT, robotJoints[i]) >= 0)
            ++robotDof;
    for (int i = 0; i < 4; ++i)
        if (mj_name2id(mModel, mjOBJ_JOINT, gripperJoints[i]) >= 0)
            ++gripperDof;

    const char* toolName = "none";
    if (mj_name2id(mModel, mjOBJ_BODY, "hande") >= 0)
        toolName = "Robotiq Hand-E";
    else if (mj_name2id(mModel, mjOBJ_BODY, "gripper_base") >= 0)
        toolName = "Generic gripper";

    std::printf("=== Simulator Info ===\n");
    std::printf("  Robot      : %s\n", modelName);
    std::printf("  Tool       : %s\n", toolName);
    std::printf("  DoF\n");
    std::printf("    Robot    : %d\n", robotDof);
    std::printf("    Gripper  : %d\n", gripperDof);
    std::printf("  Totals\n");
    std::printf("    Joints   : %d\n", mModel->njnt);
    std::printf("    Cameras  : %d", mModel->ncam);
    if (mModel->ncam > 0)
    {
        std::printf(" (");
        for (int i = 0; i < mModel->ncam; ++i)
        {
            const char* name = mj_id2name(mModel, mjOBJ_CAMERA, i);
            if (i > 0) std::printf(", ");
            std::printf("%s", name ? name : "unnamed");
        }
        std::printf(")");
    }
    std::printf("\n");
    std::printf("  Viewer     : %s\n", mViewer ? "enabled" : "disabled");
    if (mPoseApplied)
        std::printf("  Pose       : %s\n", mPoseAppliedName.c_str());
    else
        std::printf("  Pose       : none\n");
}

// Return the joint count for the currently loaded model.
int dxRobotSimulator::numJoints() const
{
    return mModel ? mModel->njnt : 0;
}

// Return the actuator count for the currently loaded model.
int dxRobotSimulator::numActuators() const
{
    return mModel ? mModel->nu : 0;
}

// Get a copy of generalized positions (qpos) from the current state.
std::vector<double> dxRobotSimulator::getQpos() const
{
    std::vector<double> out;
    if (!mModel || !mData) return out;
    out.assign(mData->qpos, mData->qpos + mModel->nq);
    return out;
}

// Get a copy of generalized velocities (qvel) from the current state.
std::vector<double> dxRobotSimulator::getQvel() const
{
    std::vector<double> out;
    if (!mModel || !mData) return out;
    out.assign(mData->qvel, mData->qvel + mModel->nv);
    return out;
}

// Get a copy of current actuator controls.
std::vector<double> dxRobotSimulator::getCtrl() const
{
    std::vector<double> out;
    if (!mModel || !mData) return out;
    out.assign(mData->ctrl, mData->ctrl + mModel->nu);
    return out;
}

// Set actuator control by index with bounds checks.
bool dxRobotSimulator::setCtrl(int actuatorIndex, double value)
{
    if (!mModel || !mData) return false;
    if (actuatorIndex < 0 || actuatorIndex >= mModel->nu) return false;
    mData->ctrl[actuatorIndex] = value;
    return true;
}

// Set actuator control by name; returns false if not found.
bool dxRobotSimulator::setCtrlByName(const std::string& actuatorName, double value)
{
    if (!mModel || !mData) return false;
    const int id = mj_name2id(mModel, mjOBJ_ACTUATOR, actuatorName.c_str());
    if (id < 0) return false;
    mData->ctrl[id] = value;
    return true;
}

// Set actuator controls from a vector, applied in actuator index order.
void dxRobotSimulator::setCtrlTargets(const std::vector<double>& targets)
{
    if (!mModel || !mData) return;
    const int count = static_cast<int>(targets.size());
    const int limit = (count < mModel->nu) ? count : mModel->nu;
    for (int i = 0; i < limit; ++i)
        mData->ctrl[i] = targets[i];
}

// Release viewer, data, and model in a safe order.
void dxRobotSimulator::shutdown()
{
    if (mViewer)
    {
        // Viewer may hold GL resources tied to the model/data.
        mViewer.reset();
    }

    if (mData)
    {
        // Free MuJoCo data before deleting the model.
        mj_deleteData(mData);
        mData = nullptr;
    }
    if (mModel)
    {
        mj_deleteModel(mModel);
        mModel = nullptr;
    }
}

// Apply a named pose saved in the model (e.g., HOME_POS = 90-90 joint start).
bool dxRobotSimulator::applyPoseByName(const char* poseName)
{
    if (!mModel || !mData || !poseName) return false;
    const int poseId = mj_name2id(mModel, mjOBJ_KEY, poseName);
    if (poseId < 0) return false;
    mj_resetDataKeyframe(mModel, mData, poseId);
    mPoseApplied = true;
    mPoseAppliedName = poseName;
    printPoseSummary(poseName);
    return true;
}

// Print a concise joint-angle summary (degrees) after a pose change.
void dxRobotSimulator::printPoseSummary(const char* poseName) const
{
    if (!mModel || !mData) return;
    const char* jointNames[6] = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };
    const char* shortNames[6] = {
        "shoulder_pan",
        "shoulder_lift",
        "elbow",
        "wrist_1",
        "wrist_2",
        "wrist_3"
    };

    std::printf("  Pose       : %s\n", poseName ? poseName : "(unnamed)");
    std::printf("  curr_q_pos (deg) = (");
    bool first = true;
    for (int i = 0; i < 6; ++i)
    {
        const int jid = mj_name2id(mModel, mjOBJ_JOINT, jointNames[i]);
        if (jid < 0) continue;
        const int qposAdr = mModel->jnt_qposadr[jid];
        if (qposAdr < 0 || qposAdr >= mModel->nq) continue;
        const double deg = mData->qpos[qposAdr] * 180.0 / 3.141592653589793;
        if (!first) std::printf(", ");
        std::printf("%.2f", deg);
        first = false;
    }
    std::printf(")\n");
}

// Print the current 6-DOF arm pose in degrees.
void dxRobotSimulator::printCurrentPoseDegrees() const
{
    if (!mModel || !mData)
    {
        std::printf("  curr_q_pos (deg) = (unavailable)\n");
        return;
    }
    const char* jointNames[6] = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };

    std::printf("  curr_q_pos (deg) = (");
    bool first = true;
    for (int i = 0; i < 6; ++i)
    {
        const int jid = mj_name2id(mModel, mjOBJ_JOINT, jointNames[i]);
        if (jid < 0) continue;
        const int qposAdr = mModel->jnt_qposadr[jid];
        if (qposAdr < 0 || qposAdr >= mModel->nq) continue;
        const double deg = mData->qpos[qposAdr] * 180.0 / 3.141592653589793;
        if (!first) std::printf(", ");
        std::printf("%.2f", deg);
        first = false;
    }
    std::printf(")\n");
}

