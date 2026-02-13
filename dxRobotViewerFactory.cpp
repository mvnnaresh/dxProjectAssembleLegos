#include "dxRobotViewerFactory.h"

#include <cstdio>
#include <new>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

dxRobotViewerFactory::dxRobotViewerFactory(const std::string& modelPath)
    : m_modelPath(modelPath)
{
}

dxRobotViewerFactory::dxRobotViewerFactory(mjModel* model, mjData* data)
    : m_modelPath(), m_model(model), m_data(data), m_ownsModelData(false)
{
}

dxRobotViewerFactory::~dxRobotViewerFactory()
{
    shutdown();
}

int dxRobotViewerFactory::run()
{
    if (!initGLFW()) return 1;
    if (!createWindow()) return 2;
    if (!m_model || !m_data)
    {
        if (!loadModel()) return 3;
    }

    initVisuals();
    m_initialized = true;
    mainLoop();
    shutdown();
    return 0;
}

bool dxRobotViewerFactory::init()
{
    if (m_initialized) return true;
    if (!m_model || !m_data) return false;
    if (!initGLFW()) return false;
    if (!createWindow()) return false;
    initVisuals();
    m_initialized = true;
    return true;
}

void dxRobotViewerFactory::renderOnce()
{
    if (!m_initialized || !m_window) return;
    renderFrame();
    glfwSwapBuffers(m_window);
    glfwPollEvents();
}

bool dxRobotViewerFactory::shouldClose() const
{
    if (!m_window) return true;
    return glfwWindowShouldClose(m_window) != 0;
}

bool dxRobotViewerFactory::initGLFW()
{
    glfwSetErrorCallback(&dxRobotViewerFactory::glfwErrorCallback);
    if (!glfwInit())
    {
        std::fprintf(stderr, "Failed to initialise GLFW.\n");
        return false;
    }
    return true;
}

bool dxRobotViewerFactory::createWindow()
{
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);

    m_window = glfwCreateWindow(m_width, m_height, "DexMan Viewer", nullptr, nullptr);
    if (!m_window)
    {
        std::fprintf(stderr, "Failed to create GLFW window.\n");
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);

    glfwSetWindowUserPointer(m_window, this);

    glfwSetKeyCallback(m_window, &dxRobotViewerFactory::keyCallback);
    glfwSetMouseButtonCallback(m_window, &dxRobotViewerFactory::mouseButtonCallback);
    glfwSetCursorPosCallback(m_window, &dxRobotViewerFactory::cursorPosCallback);
    glfwSetScrollCallback(m_window, &dxRobotViewerFactory::scrollCallback);

    return true;
}

bool dxRobotViewerFactory::loadModel()
{
    char err[1024] = { 0 };
    m_model = mj_loadXML(m_modelPath.c_str(), nullptr, err, sizeof(err));
    if (!m_model)
    {
        std::fprintf(stderr, "mj_loadXML failed for '%s'\nError: %s\n",
                     m_modelPath.c_str(), err);
        return false;
    }

    m_data = mj_makeData(m_model);
    if (!m_data)
    {
        std::fprintf(stderr, "mj_makeData failed.\n");
        return false;
    }

    std::printf("MuJoCo version: %s\n", mj_versionString());
    std::printf("Loaded model: %s\n", m_modelPath.c_str());
    return true;
}

void dxRobotViewerFactory::initVisuals()
{
    m_cam = new (std::nothrow) mjvCamera;
    m_opt = new (std::nothrow) mjvOption;
    m_scn = new (std::nothrow) mjvScene;
    m_con = new (std::nothrow) mjrContext;

    if (!m_cam || !m_opt || !m_scn || !m_con)
    {
        std::fprintf(stderr, "Failed to allocate visualisation structs.\n");
        return;
    }

    mjv_defaultCamera(m_cam);
    mjv_defaultOption(m_opt);
    mjv_defaultScene(m_scn);
    mjr_defaultContext(m_con);

    m_opt->frame = mjFRAME_WORLD;

    mjv_makeScene(m_model, m_scn, 4000);
    mjr_makeContext(m_model, m_con, mjFONTSCALE_150);

    // Reasonable default camera
    m_cam->azimuth = 90.0;
    m_cam->elevation = -20.0;
    m_cam->distance = 3.0;
    m_cam->lookat[0] = 0.0;
    m_cam->lookat[1] = 0.0;
    m_cam->lookat[2] = 0.5;

    mj_forward(m_model, m_data);
}

void dxRobotViewerFactory::mainLoop()
{
    if (!m_window || !m_model || !m_data || !m_cam || !m_opt || !m_scn || !m_con)
        return;

    double lastWall = glfwGetTime();
    double simAccum = 0.0;

    while (!glfwWindowShouldClose(m_window))
    {
        const double now = glfwGetTime();
        double dtWall = now - lastWall;
        lastWall = now;

        if (dtWall > 0.1) dtWall = 0.1;
        simAccum += dtWall;

        if (!m_paused)
        {
            const double dtSim = m_model->opt.timestep;
            while (simAccum >= dtSim)
            {
                mj_step(m_model, m_data);
                simAccum -= dtSim;
            }
        }
        else if (m_stepOnce)
        {
            mj_step(m_model, m_data);
            m_stepOnce = false;
        }

        renderFrame();

        glfwSwapBuffers(m_window);
        glfwPollEvents();
    }
}

void dxRobotViewerFactory::renderFrame()
{
    int w = 0, h = 0;
    glfwGetFramebufferSize(m_window, &w, &h);
    mjrRect viewport = { 0, 0, w, h };

    mjv_updateScene(m_model, m_data, m_opt, nullptr, m_cam, mjCAT_ALL, m_scn);
    mjr_render(viewport, m_scn, m_con);
}

void dxRobotViewerFactory::shutdown()
{
    if (m_shutdownDone) return;
    m_shutdownDone = true;

    if (m_con && m_model) mjr_freeContext(m_con);
    if (m_scn && m_model) mjv_freeScene(m_scn);

    delete m_con;
    m_con = nullptr;
    delete m_scn;
    m_scn = nullptr;
    delete m_opt;
    m_opt = nullptr;
    delete m_cam;
    m_cam = nullptr;

    if (m_ownsModelData)
    {
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

    if (m_window)
    {
        glfwDestroyWindow(m_window);
        m_window = nullptr;
    }

    glfwTerminate();
}

void dxRobotViewerFactory::glfwErrorCallback(int error, const char* desc)
{
    std::fprintf(stderr, "GLFW error %d: %s\n", error, desc ? desc : "(null)");
}

dxRobotViewerFactory* dxRobotViewerFactory::self(GLFWwindow* w)
{
    return static_cast<dxRobotViewerFactory*>(glfwGetWindowUserPointer(w));
}

void dxRobotViewerFactory::keyCallback(GLFWwindow* w, int key, int, int action, int)
{
    dxRobotViewerFactory* s = self(w);
    if (!s) return;
    if (action == GLFW_RELEASE) return;

    if (key == GLFW_KEY_ESCAPE)
    {
        glfwSetWindowShouldClose(w, GLFW_TRUE);
        return;
    }

    // Handy dev controls
    if (key == GLFW_KEY_SPACE) s->m_paused = !s->m_paused;
    if (key == GLFW_KEY_PERIOD && s->m_paused) s->m_stepOnce = true;

    if (key == GLFW_KEY_R && s->m_model && s->m_data)
    {
        mj_resetData(s->m_model, s->m_data);
        mj_forward(s->m_model, s->m_data);
    }
}

void dxRobotViewerFactory::mouseButtonCallback(GLFWwindow* w, int button, int action, int)
{
    dxRobotViewerFactory* s = self(w);
    if (!s) return;

    if (button == GLFW_MOUSE_BUTTON_LEFT)   s->m_btnLeft = (action == GLFW_PRESS);
    if (button == GLFW_MOUSE_BUTTON_MIDDLE) s->m_btnMiddle = (action == GLFW_PRESS);
    if (button == GLFW_MOUSE_BUTTON_RIGHT)  s->m_btnRight = (action == GLFW_PRESS);

    glfwGetCursorPos(w, &s->m_lastX, &s->m_lastY);
}

void dxRobotViewerFactory::cursorPosCallback(GLFWwindow* w, double xpos, double ypos)
{
    dxRobotViewerFactory* s = self(w);
    if (!s || !s->m_model || !s->m_scn || !s->m_cam) return;

    const double dx = xpos - s->m_lastX;
    const double dy = ypos - s->m_lastY;
    s->m_lastX = xpos;
    s->m_lastY = ypos;

    if (!s->m_btnLeft && !s->m_btnMiddle && !s->m_btnRight) return;

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(w, &width, &height);
    if (height <= 0) return;

    const bool shift =
        (glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
        (glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse act = mjMOUSE_NONE;
    if (s->m_btnRight)
        act = shift ? mjMOUSE_MOVE_V : mjMOUSE_MOVE_H;
    else if (s->m_btnMiddle)
        act = mjMOUSE_ZOOM;
    else if (s->m_btnLeft)
        act = shift ? mjMOUSE_ROTATE_V : mjMOUSE_ROTATE_H;

    mjv_moveCamera(s->m_model, act,
                   (mjtNum)(dx / static_cast<double>(height)),
                   (mjtNum)(dy / static_cast<double>(height)),
                   s->m_scn, s->m_cam);
}

void dxRobotViewerFactory::scrollCallback(GLFWwindow* w, double, double yoffset)
{
    dxRobotViewerFactory* s = self(w);
    if (!s || !s->m_model || !s->m_scn || !s->m_cam) return;

    // Scroll zoom
    const mjtNum zoom = (mjtNum)(-0.05 * yoffset);
    mjv_moveCamera(s->m_model, mjMOUSE_ZOOM, 0.0, zoom, s->m_scn, s->m_cam);
}
