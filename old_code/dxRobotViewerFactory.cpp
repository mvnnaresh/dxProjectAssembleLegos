#include "dxRobotViewerFactory.h"

#include <new>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

dxRobotViewerFactory::dxRobotViewerFactory(const std::string& modelPath)
    : mModelPath(modelPath)
{
}

dxRobotViewerFactory::dxRobotViewerFactory(mjModel* model, mjData* data)
    : mModelPath(), mModel(model), mData(data), mOwnsModelData(false)
{
}

dxRobotViewerFactory::~dxRobotViewerFactory()
{
    shutdown();
}

int dxRobotViewerFactory::run()
{
    if (!initGLFW())
    {
        return 1;
    }
    if (!createWindow())
    {
        return 2;
    }
    if (!mModel || !mData)
    {
        if (!loadModel())
        {
            return 3;
        }
    }

    initVisuals();
    mInitialized = true;
    mainLoop();
    shutdown();
    return 0;
}

bool dxRobotViewerFactory::init()
{
    if (mInitialized)
    {
        return true;
    }
    if (!mModel || !mData)
    {
        return false;
    }
    if (!initGLFW())
    {
        return false;
    }
    if (!createWindow())
    {
        return false;
    }
    initVisuals();
    mInitialized = true;
    return true;
}

void dxRobotViewerFactory::renderOnce()
{
    if (!mInitialized || !mWindow)
    {
        return;
    }
    renderFrame();
    glfwSwapBuffers(mWindow);
    glfwPollEvents();
}

bool dxRobotViewerFactory::shouldClose() const
{
    if (!mWindow)
    {
        return true;
    }
    return glfwWindowShouldClose(mWindow) != 0;
}

bool dxRobotViewerFactory::initGLFW()
{
    glfwSetErrorCallback(&dxRobotViewerFactory::glfwErrorCallback);
    if (!glfwInit())
    {
        return false;
    }
    return true;
}

bool dxRobotViewerFactory::createWindow()
{
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);

    mWindow = glfwCreateWindow(mWidth, mHeight, "DexMan Viewer", nullptr, nullptr);
    if (!mWindow)
    {
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(mWindow);
    glfwSwapInterval(1);

    glfwSetWindowUserPointer(mWindow, this);

    glfwSetKeyCallback(mWindow, &dxRobotViewerFactory::keyCallback);
    glfwSetMouseButtonCallback(mWindow, &dxRobotViewerFactory::mouseButtonCallback);
    glfwSetCursorPosCallback(mWindow, &dxRobotViewerFactory::cursorPosCallback);
    glfwSetScrollCallback(mWindow, &dxRobotViewerFactory::scrollCallback);

    return true;
}

bool dxRobotViewerFactory::loadModel()
{
    char err[1024] = { 0 };
    mModel = mj_loadXML(mModelPath.c_str(), nullptr, err, sizeof(err));
    if (!mModel)
    {
        return false;
    }

    mData = mj_makeData(mModel);
    if (!mData)
    {
        return false;
    }
    return true;
}

void dxRobotViewerFactory::initVisuals()
{
    mCam = new (std::nothrow) mjvCamera;
    mOpt = new (std::nothrow) mjvOption;
    mScn = new (std::nothrow) mjvScene;
    mCon = new (std::nothrow) mjrContext;

    if (!mCam || !mOpt || !mScn || !mCon)
    {
        return;
    }

    mjv_defaultCamera(mCam);
    mjv_defaultOption(mOpt);
    mjv_defaultScene(mScn);
    mjr_defaultContext(mCon);

    mOpt->frame = mjFRAME_WORLD;

    mjv_makeScene(mModel, mScn, 4000);
    mjr_makeContext(mModel, mCon, mjFONTSCALE_150);

    // Reasonable default camera
    mCam->azimuth = 90.0;
    mCam->elevation = -20.0;
    mCam->distance = 3.0;
    mCam->lookat[0] = 0.0;
    mCam->lookat[1] = 0.0;
    mCam->lookat[2] = 0.5;

    mj_forward(mModel, mData);
}

void dxRobotViewerFactory::mainLoop()
{
    if (!mWindow || !mModel || !mData || !mCam || !mOpt || !mScn || !mCon)
    {
        return;
    }

    double lastWall = glfwGetTime();
    double simAccum = 0.0;

    while (!glfwWindowShouldClose(mWindow))
    {
        const double now = glfwGetTime();
        double dtWall = now - lastWall;
        lastWall = now;

        if (dtWall > 0.1)
        {
            dtWall = 0.1;
        }
        simAccum += dtWall;

        if (!mPaused)
        {
            const double dtSim = mModel->opt.timestep;
            while (simAccum >= dtSim)
            {
                mj_step(mModel, mData);
                simAccum -= dtSim;
            }
        }
        else if (mStepOnce)
        {
            mj_step(mModel, mData);
            mStepOnce = false;
        }

        renderFrame();

        glfwSwapBuffers(mWindow);
        glfwPollEvents();
    }
}

void dxRobotViewerFactory::renderFrame()
{
    int w = 0, h = 0;
    glfwGetFramebufferSize(mWindow, &w, &h);
    mjrRect viewport = { 0, 0, w, h };

    mjv_updateScene(mModel, mData, mOpt, nullptr, mCam, mjCAT_ALL, mScn);
    mjr_render(viewport, mScn, mCon);
}

void dxRobotViewerFactory::shutdown()
{
    if (mShutdownDone)
    {
        return;
    }
    mShutdownDone = true;

    if (mCon && mModel)
    {
        mjr_freeContext(mCon);
    }
    if (mScn && mModel)
    {
        mjv_freeScene(mScn);
    }

    delete mCon;
    mCon = nullptr;
    delete mScn;
    mScn = nullptr;
    delete mOpt;
    mOpt = nullptr;
    delete mCam;
    mCam = nullptr;

    if (mOwnsModelData)
    {
        if (mData)
        {
            mj_deleteData(mData);
            mData = nullptr;
        }
        if (mModel)
        {
            mj_deleteModel(mModel);
            mModel = nullptr;
        }
    }

    if (mWindow)
    {
        glfwDestroyWindow(mWindow);
        mWindow = nullptr;
    }

    glfwTerminate();
}

void dxRobotViewerFactory::glfwErrorCallback(int error, const char* desc)
{
    (void)error;
    (void)desc;
}

dxRobotViewerFactory* dxRobotViewerFactory::self(GLFWwindow* w)
{
    return static_cast<dxRobotViewerFactory*>(glfwGetWindowUserPointer(w));
}

void dxRobotViewerFactory::keyCallback(GLFWwindow* w, int key, int, int action, int)
{
    dxRobotViewerFactory* s = self(w);
    if (!s)
    {
        return;
    }
    if (action == GLFW_RELEASE)
    {
        return;
    }

    if (key == GLFW_KEY_ESCAPE)
    {
        glfwSetWindowShouldClose(w, GLFW_TRUE);
        return;
    }

    // Handy dev controls
    if (key == GLFW_KEY_SPACE)
    {
        s->mPaused = !s->mPaused;
    }
    if (key == GLFW_KEY_PERIOD && s->mPaused)
    {
        s->mStepOnce = true;
    }

    if (key == GLFW_KEY_R && s->mModel && s->mData)
    {
        mj_resetData(s->mModel, s->mData);
        mj_forward(s->mModel, s->mData);
    }
}

void dxRobotViewerFactory::mouseButtonCallback(GLFWwindow* w, int button, int action, int)
{
    dxRobotViewerFactory* s = self(w);
    if (!s)
    {
        return;
    }

    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        s->mBtnLeft = (action == GLFW_PRESS);
    }
    if (button == GLFW_MOUSE_BUTTON_MIDDLE)
    {
        s->mBtnMiddle = (action == GLFW_PRESS);
    }
    if (button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        s->mBtnRight = (action == GLFW_PRESS);
    }

    glfwGetCursorPos(w, &s->mLastX, &s->mLastY);
}

void dxRobotViewerFactory::cursorPosCallback(GLFWwindow* w, double xpos, double ypos)
{
    dxRobotViewerFactory* s = self(w);
    if (!s || !s->mModel || !s->mScn || !s->mCam)
    {
        return;
    }

    const double dx = xpos - s->mLastX;
    const double dy = ypos - s->mLastY;
    s->mLastX = xpos;
    s->mLastY = ypos;

    if (!s->mBtnLeft && !s->mBtnMiddle && !s->mBtnRight)
    {
        return;
    }

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(w, &width, &height);
    if (height <= 0)
    {
        return;
    }

    const bool shift =
        (glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
        (glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse act = mjMOUSE_NONE;
    if (s->mBtnRight)
    {
        act = shift ? mjMOUSE_MOVE_V : mjMOUSE_MOVE_H;
    }
    else if (s->mBtnMiddle)
    {
        act = mjMOUSE_ZOOM;
    }
    else if (s->mBtnLeft)
    {
        act = shift ? mjMOUSE_ROTATE_V : mjMOUSE_ROTATE_H;
    }

    mjv_moveCamera(s->mModel, act,
                   (mjtNum)(dx / static_cast<double>(height)),
                   (mjtNum)(dy / static_cast<double>(height)),
                   s->mScn, s->mCam);
}

void dxRobotViewerFactory::scrollCallback(GLFWwindow* w, double, double yoffset)
{
    dxRobotViewerFactory* s = self(w);
    if (!s || !s->mModel || !s->mScn || !s->mCam)
    {
        return;
    }

    // Scroll zoom
    const mjtNum zoom = (mjtNum)(-0.05 * yoffset);
    mjv_moveCamera(s->mModel, mjMOUSE_ZOOM, 0.0, zoom, s->mScn, s->mCam);
}


