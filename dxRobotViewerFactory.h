#pragma once

#pragma once
#include <string>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

// struct GLFWwindow;
//
// struct mjModel;
// struct mjData;
// struct mjvCamera;
// struct mjvOption;
// struct mjvScene;
// struct mjrContext;

class dxRobotViewerFactory
{
public:
    explicit dxRobotViewerFactory(const std::string& modelPath);
    dxRobotViewerFactory(mjModel* model, mjData* data);
    ~dxRobotViewerFactory();

    dxRobotViewerFactory(const dxRobotViewerFactory&) = delete;
    dxRobotViewerFactory& operator=(const dxRobotViewerFactory&) = delete;

    int run();
    bool init();
    void renderOnce();
    bool shouldClose() const;

private:
    bool initGLFW();
    bool createWindow();
    bool loadModel();
    void initVisuals();
    void mainLoop();
    void renderFrame();
    void shutdown();

    // GLFW callbacks
    static void glfwErrorCallback(int error, const char* desc);
    static void keyCallback(GLFWwindow* w, int key, int scancode, int action, int mods);
    static void mouseButtonCallback(GLFWwindow* w, int button, int action, int mods);
    static void cursorPosCallback(GLFWwindow* w, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* w, double xoffset, double yoffset);

    static dxRobotViewerFactory* self(GLFWwindow* w);

private:
    std::string m_modelPath;

    GLFWwindow* m_window = nullptr;
    int m_width = 1200;
    int m_height = 900;

    mjModel* m_model = nullptr;
    mjData*  m_data = nullptr;

    mjvCamera*  m_cam = nullptr;
    mjvOption*  m_opt = nullptr;
    mjvScene*   m_scn = nullptr;
    mjrContext* m_con = nullptr;

    bool m_paused = false;
    bool m_stepOnce = false;

    bool m_btnLeft = false;
    bool m_btnMiddle = false;
    bool m_btnRight = false;
    double m_lastX = 0.0;
    double m_lastY = 0.0;

    bool m_ownsModelData = true;
    bool m_initialized = false;

    bool m_shutdownDone = false;
};
