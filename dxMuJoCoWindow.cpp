#include "dxMuJoCoWindow.h"

#include <cstdio>

#include <QMouseEvent>
#include <QWheelEvent>
#include <QOpenGLFunctions>

dxMuJoCoWindow::dxMuJoCoWindow(QWindow* parent)
    : QOpenGLWindow(NoPartialUpdate, parent)
{
    mjv_defaultCamera(&mCam);
    mjv_defaultOption(&mOpt);
    mjv_defaultScene(&mScn);
    mjr_defaultContext(&mCon);
}

dxMuJoCoWindow::~dxMuJoCoWindow()
{
    shutdownVisuals();
    shutdownModel();
}

bool dxMuJoCoWindow::loadModel(const std::string& modelPath)
{
    mModelPath = modelPath;
    shutdownModel();

    char err[1024] = { 0 };
    mModel = mj_loadXML(modelPath.c_str(), nullptr, err, sizeof(err));
    if (!mModel)
    {
        std::fprintf(stderr, "dxMuJoCoWindow: mj_loadXML failed for '%s'\nError: %s\n",
                     modelPath.c_str(), err);
        return false;
    }

    mData = mj_makeData(mModel);
    if (!mData)
    {
        std::fprintf(stderr, "dxMuJoCoWindow: mj_makeData failed.\n");
        shutdownModel();
        return false;
    }

    mOwnsModelData = true;
    mj_forward(mModel, mData);

    mNeedsVisualInit = true;
    update();
    return true;
}

void dxMuJoCoWindow::setModel(mjModel* model, mjData* data, bool ownsModelData)
{
    shutdownModel();
    mModel = model;
    mData = data;
    mOwnsModelData = ownsModelData;
    if (mModel && mData)
        mj_forward(mModel, mData);

    mNeedsVisualInit = true;
    update();
}

void dxMuJoCoWindow::reset()
{
    if (!mModel || !mData) return;
    mj_resetData(mModel, mData);
    mj_forward(mModel, mData);
    update();
}

void dxMuJoCoWindow::initializeGL()
{
    if (mModel)
        initVisuals();
}

void dxMuJoCoWindow::paintGL()
{
    if (!mModel || !mData) return;
    if (!mVisualsReady && mNeedsVisualInit)
        initVisuals();
    if (!mVisualsReady) return;

    if (QOpenGLFunctions* f = context() ? context()->functions() : nullptr)
    {
        f->glClearColor(0.05f, 0.06f, 0.07f, 1.0f);
        f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    mj_forward(mModel, mData);

    const int w = width() * devicePixelRatio();
    const int h = height() * devicePixelRatio();
    mjrRect viewport = { 0, 0, w, h };

    mjv_updateScene(mModel, mData, &mOpt, nullptr, &mCam, mjCAT_ALL, &mScn);
    mjr_render(viewport, &mScn, &mCon);
}

void dxMuJoCoWindow::resizeGL(int w, int h)
{
    Q_UNUSED(w);
    Q_UNUSED(h);
    update();
}

void dxMuJoCoWindow::mousePressEvent(QMouseEvent* event)
{
    if (!event) return;
    if (event->button() == Qt::LeftButton) mBtnLeft = true;
    if (event->button() == Qt::MiddleButton) mBtnMiddle = true;
    if (event->button() == Qt::RightButton) mBtnRight = true;
    const QPointF p = event->localPos();
    mLastX = p.x();
    mLastY = p.y();
}

void dxMuJoCoWindow::mouseReleaseEvent(QMouseEvent* event)
{
    if (!event) return;
    if (event->button() == Qt::LeftButton) mBtnLeft = false;
    if (event->button() == Qt::MiddleButton) mBtnMiddle = false;
    if (event->button() == Qt::RightButton) mBtnRight = false;
}

void dxMuJoCoWindow::mouseMoveEvent(QMouseEvent* event)
{
    if (!event || !mModel || !mVisualsReady) return;

    const QPointF p = event->localPos();
    const double xpos = p.x();
    const double ypos = p.y();
    const double dx = xpos - mLastX;
    const double dy = ypos - mLastY;
    mLastX = xpos;
    mLastY = ypos;

    if (!mBtnLeft && !mBtnMiddle && !mBtnRight) return;

    const bool shift = event->modifiers().testFlag(Qt::ShiftModifier);

    mjtMouse act = mjMOUSE_NONE;
    if (mBtnRight)
        act = shift ? mjMOUSE_MOVE_V : mjMOUSE_MOVE_H;
    else if (mBtnMiddle)
        act = mjMOUSE_ZOOM;
    else if (mBtnLeft)
        act = shift ? mjMOUSE_ROTATE_V : mjMOUSE_ROTATE_H;

    const double dpr = devicePixelRatio();
    const double h = std::max(1.0, static_cast<double>(height() * dpr));

    mjv_moveCamera(mModel, act,
                   static_cast<mjtNum>(dx / h),
                   static_cast<mjtNum>(dy / h),
                   &mScn, &mCam);
    update();
}

void dxMuJoCoWindow::wheelEvent(QWheelEvent* event)
{
    if (!event || !mModel || !mVisualsReady) return;
    const QPointF angle = event->angleDelta();
    const QPointF pixel = event->pixelDelta();
    const double steps = (std::abs(pixel.y()) > 0.0) ? (pixel.y() / 120.0) : (angle.y() / 120.0);
    const mjtNum zoom = static_cast<mjtNum>(-0.03 * steps);
    mjv_moveCamera(mModel, mjMOUSE_ZOOM, 0.0, zoom, &mScn, &mCam);
    update();
}

void dxMuJoCoWindow::initVisuals()
{
    if (!mModel) return;

    shutdownVisuals();

    mjv_defaultCamera(&mCam);
    mjv_defaultOption(&mOpt);
    mjv_defaultScene(&mScn);
    mjr_defaultContext(&mCon);

    mOpt.frame = mjFRAME_WORLD;

    mjv_makeScene(mModel, &mScn, 4000);
    mjr_makeContext(mModel, &mCon, mjFONTSCALE_150);

    mCam.azimuth = 90.0;
    mCam.elevation = -20.0;
    mCam.distance = 3.0;
    mCam.lookat[0] = 0.0;
    mCam.lookat[1] = 0.0;
    mCam.lookat[2] = 0.5;

    mVisualsReady = true;
    mNeedsVisualInit = false;
}

void dxMuJoCoWindow::shutdownVisuals()
{
    if (!mVisualsReady) return;
    mjr_freeContext(&mCon);
    mjv_freeScene(&mScn);
    mVisualsReady = false;
}

void dxMuJoCoWindow::shutdownModel()
{
    if (!mOwnsModelData) return;

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
    mOwnsModelData = false;
}
