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
    std::string mModelPath;

    GLFWwindow* mWindow = nullptr;
    int mWidth = 1200;
    int mHeight = 900;

    mjModel* mModel = nullptr;
    mjData*  mData = nullptr;

    mjvCamera*  mCam = nullptr;
    mjvOption*  mOpt = nullptr;
    mjvScene*   mScn = nullptr;
    mjrContext* mCon = nullptr;

    bool mPaused = false;
    bool mStepOnce = false;

    bool mBtnLeft = false;
    bool mBtnMiddle = false;
    bool mBtnRight = false;
    double mLastX = 0.0;
    double mLastY = 0.0;

    bool mOwnsModelData = true;
    bool mInitialized = false;

    bool mShutdownDone = false;
};


