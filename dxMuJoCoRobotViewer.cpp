#include "dxMuJoCoRobotViewer.h"

#include <algorithm>
#include <cmath>

#include <QDebug>
#include <QMouseEvent>
#include <QOpenGLFunctions>
#include <QWheelEvent>

dxMuJoCoRobotViewer::dxMuJoCoRobotViewer(QWindow* parent)
    : QOpenGLWindow(NoPartialUpdate, parent)
{
    mjv_defaultCamera(&mCam);
    mjv_defaultOption(&mOpt);
    mjv_defaultScene(&mScn);
    mjr_defaultContext(&mCon);
}

dxMuJoCoRobotViewer::~dxMuJoCoRobotViewer()
{
    shutdownVisuals();
    shutdownModel();
}

bool dxMuJoCoRobotViewer::loadModel(const std::string& modelPath)
{
    mModelPath = modelPath;
    shutdownModel();

    char err[1024] = { 0 };
    mModel = mj_loadXML(modelPath.c_str(), nullptr, err, sizeof(err));
    if (!mModel)
    {
        return false;
    }

    mData = mj_makeData(mModel);
    if (!mData)
    {
        shutdownModel();
        return false;
    }

    mOwnsModel = true;
    mOwnsData = true;
    mj_forward(mModel, mData);
    mStreamCamera.setModel(mModel, mData);

    mNeedsVisualInit = true;
    update();
    return true;
}

void dxMuJoCoRobotViewer::setModel(mjModel* model)
{
    shutdownModel();
    mModel = model;
    if (mModel)
    {
        mData = mj_makeData(mModel);
    }
    if (!mData)
    {
        mModel = nullptr;
        mOwnsModel = false;
        mOwnsData = false;
        return;
    }
    mOwnsModel = false;
    mOwnsData = true;
    mj_forward(mModel, mData);
    mStreamCamera.setModel(mModel, mData);

    mNeedsVisualInit = true;
    update();
}

void dxMuJoCoRobotViewer::reset()
{
    if (!mModel || !mData)
    {
        return;
    }
    mj_resetData(mModel, mData);
    mj_forward(mModel, mData);
    update();
}

void dxMuJoCoRobotViewer::drawTrajectory(const std::vector<std::array<double, 3>>& points)
{
    mTrajectoryPath = points;
    if (mTrajectoryPath.size() > mMaxPathPoints)
    {
        mTrajectoryPath.resize(mMaxPathPoints);
    }
    update();
}

void dxMuJoCoRobotViewer::drawFrames(const std::vector<std::array<double, 12>>& frames)
{
    mDebugFrames = frames;
    update();
}

void dxMuJoCoRobotViewer::runWithContext(const std::function<void()>& fn)
{
    mPendingContextFn = fn;
    update();
}

void dxMuJoCoRobotViewer::applyState(const dxMuJoCoRobotState& state)
{
    if (!mModel || !mData)
    {
        return;
    }
    if (static_cast<int>(state.worldQpos.size()) == mModel->nq)
    {
        for (int i = 0; i < mModel->nq; ++i)
        {
            mData->qpos[i] = state.worldQpos[static_cast<size_t>(i)];
        }
    }
    else if (!state.jointConf.empty())
    {
        int jointIndex = 0;
        for (int jid = 0; jid < mModel->njnt; ++jid)
        {
            const int type = mModel->jnt_type[jid];
            if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
            {
                continue;
            }
            if (jointIndex >= static_cast<int>(state.jointConf.size()))
            {
                break;
            }
            const int qposAdr = mModel->jnt_qposadr[jid];
            if (qposAdr >= 0 && qposAdr < mModel->nq)
            {
                mData->qpos[qposAdr] = state.jointConf[static_cast<size_t>(jointIndex)];
            }
            ++jointIndex;
        }
    }
    if (!state.jointVel.empty())
    {
        int jointIndex = 0;
        for (int jid = 0; jid < mModel->njnt; ++jid)
        {
            const int type = mModel->jnt_type[jid];
            if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
            {
                continue;
            }
            if (jointIndex >= static_cast<int>(state.jointVel.size()))
            {
                break;
            }
            const int dofAdr = mModel->jnt_dofadr[jid];
            if (dofAdr >= 0 && dofAdr < mModel->nv)
            {
                mData->qvel[dofAdr] = state.jointVel[static_cast<size_t>(jointIndex)];
            }
            ++jointIndex;
        }
    }
    if (static_cast<int>(state.actuatorInput.size()) == mModel->nu)
    {
        for (int i = 0; i < mModel->nu; ++i)
        {
            mData->ctrl[i] = state.actuatorInput[static_cast<size_t>(i)];
        }
    }
    mj_forward(mModel, mData);
    update();
}

void dxMuJoCoRobotViewer::setCameraStreamEnabled(bool enabled)
{
    mStreamEnabled = enabled;
}

void dxMuJoCoRobotViewer::setCameraStreamName(const QString& name)
{
    mStreamCameraName = name;
    mStreamCamera.setCameraName(name.toStdString());
}

void dxMuJoCoRobotViewer::setCameraStreamResolution(int width, int height)
{
    if (width > 0 && height > 0)
    {
        mStreamWidth = width;
        mStreamHeight = height;
        mStreamCamera.setResolution(width, height);
    }
}

void dxMuJoCoRobotViewer::setCameraStreamBaseBodyName(const QString& name)
{
    mStreamBaseBodyName = name;
    mStreamCamera.setBaseBodyName(name.toStdString());
}

void dxMuJoCoRobotViewer::requestPointCloudCapture()
{
    mPointCloudCapturePending = true;
}


void dxMuJoCoRobotViewer::initializeGL()
{
    if (mModel)
    {
        initVisuals();
    }
}

void dxMuJoCoRobotViewer::paintGL()
{
    if (!mModel || !mData)
    {
        return;
    }
    if (!mVisualsReady && mNeedsVisualInit)
    {
        initVisuals();
    }
    if (!mVisualsReady)
    {
        return;
    }

    if (QOpenGLFunctions* f = context() ? context()->functions() : nullptr)
    {
        f->glClearColor(0.05f, 0.06f, 0.07f, 1.0f);
        f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    const int w = width() * devicePixelRatio();
    const int h = height() * devicePixelRatio();
    mjrRect viewport = { 0, 0, w, h };

    mjv_updateScene(mModel, mData, &mOpt, nullptr, &mCam, mjCAT_ALL, &mScn);
    const float pathColor[4] = { 1.0f, 0.0f, 0.0f, 1.0f };
    appendPathGeoms(mTrajectoryPath, pathColor);
    appendFrameGeoms(mDebugFrames);
    mjr_render(viewport, &mScn, &mCon);

    if (mStreamEnabled)
    {
        dxMuJoCoRealSense::Frame frame;
        mStreamCamera.setResolution(mStreamWidth, mStreamHeight);
        if (mStreamCamera.captureRgbDepth(&mOpt, &mCon, frame))
        {
            mStreamErrorLogged = false;
            QImage image(frame.rgb.data(), frame.width, frame.height, QImage::Format_RGB888);
            emit rgbFrameReady(image.copy());
        }
        else if (!mStreamErrorLogged)
        {
            mStreamErrorLogged = true;
            qWarning() << "Camera capture failed:" << QString::fromStdString(mStreamCamera.lastError());
        }
    }

    if (mPointCloudCapturePending)
    {
        mPointCloudCapturePending = false;
        dxMuJoCoRealSense::Frame frame;
        mStreamCamera.setResolution(mStreamWidth, mStreamHeight);
        if (!mStreamBaseBodyName.isEmpty())
        {
            mStreamCamera.setBaseBodyName(mStreamBaseBodyName.toStdString());
        }
        if (mStreamCamera.captureRgbDepth(&mOpt, &mCon, frame))
        {
            std::vector<std::array<float, 3>> points;
            std::vector<std::array<unsigned char, 3>> colors;
            if (mStreamCamera.computePointCloudInBaseWithColor(frame.depth, frame.rgb, points, colors))
            {
                mPointCloudErrorLogged = false;
                emit pointCloudReady(points, colors);
            }
            else if (!mPointCloudErrorLogged)
            {
                mPointCloudErrorLogged = true;
                qWarning() << "Point cloud build failed:"
                           << QString::fromStdString(mStreamCamera.lastError());
            }
        }
        else if (!mPointCloudErrorLogged)
        {
            mPointCloudErrorLogged = true;
            qWarning() << "Point cloud capture failed:"
                       << QString::fromStdString(mStreamCamera.lastError());
        }
    }

    if (mPendingContextFn)
    {
        auto fn = std::move(mPendingContextFn);
        mPendingContextFn = nullptr;
        fn();
    }
}

void dxMuJoCoRobotViewer::resizeGL(int w, int h)
{
    Q_UNUSED(w);
    Q_UNUSED(h);
    update();
}

void dxMuJoCoRobotViewer::mousePressEvent(QMouseEvent* event)
{
    if (!event)
    {
        return;
    }
    if (event->button() == Qt::LeftButton)
    {
        mBtnLeft = true;
    }
    if (event->button() == Qt::MiddleButton)
    {
        mBtnMiddle = true;
    }
    if (event->button() == Qt::RightButton)
    {
        mBtnRight = true;
    }
    const QPointF p = event->localPos();
    mLastX = p.x();
    mLastY = p.y();
}

void dxMuJoCoRobotViewer::mouseReleaseEvent(QMouseEvent* event)
{
    if (!event)
    {
        return;
    }
    if (event->button() == Qt::LeftButton)
    {
        mBtnLeft = false;
    }
    if (event->button() == Qt::MiddleButton)
    {
        mBtnMiddle = false;
    }
    if (event->button() == Qt::RightButton)
    {
        mBtnRight = false;
    }
}

void dxMuJoCoRobotViewer::mouseMoveEvent(QMouseEvent* event)
{
    if (!event || !mModel || !mVisualsReady)
    {
        return;
    }

    const QPointF p = event->localPos();
    const double xpos = p.x();
    const double ypos = p.y();
    const double dx = xpos - mLastX;
    const double dy = ypos - mLastY;
    mLastX = xpos;
    mLastY = ypos;

    if (!mBtnLeft && !mBtnMiddle && !mBtnRight)
    {
        return;
    }

    const bool shift = event->modifiers().testFlag(Qt::ShiftModifier);

    mjtMouse act = mjMOUSE_NONE;
    if (mBtnRight)
    {
        act = shift ? mjMOUSE_MOVE_V : mjMOUSE_MOVE_H;
    }
    else if (mBtnMiddle)
    {
        act = mjMOUSE_ZOOM;
    }
    else if (mBtnLeft)
    {
        act = shift ? mjMOUSE_ROTATE_V : mjMOUSE_ROTATE_H;
    }

    const double dpr = devicePixelRatio();
    const double h = std::max(1.0, static_cast<double>(height() * dpr));

    mjv_moveCamera(mModel, act,
                   static_cast<mjtNum>(dx / h),
                   static_cast<mjtNum>(dy / h),
                   &mScn, &mCam);
    update();
}

void dxMuJoCoRobotViewer::wheelEvent(QWheelEvent* event)
{
    if (!event || !mModel || !mVisualsReady)
    {
        return;
    }
    const QPointF angle = event->angleDelta();
    const QPointF pixel = event->pixelDelta();
    const double steps = (std::abs(pixel.y()) > 0.0) ? (pixel.y() / 120.0) : (angle.y() / 120.0);
    const mjtNum zoom = static_cast<mjtNum>(-0.03 * steps);
    mjv_moveCamera(mModel, mjMOUSE_ZOOM, 0.0, zoom, &mScn, &mCam);
    update();
}

void dxMuJoCoRobotViewer::initVisuals()
{
    if (!mModel)
    {
        return;
    }

    shutdownVisuals();

    mjv_defaultCamera(&mCam);
    mjv_defaultOption(&mOpt);
    mjv_defaultScene(&mScn);
    mjr_defaultContext(&mCon);

    mOpt.frame = mjFRAME_NONE;

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

void dxMuJoCoRobotViewer::appendPathGeoms(const std::vector<std::array<double, 3>>& points, const float rgba[4])
{
    if (points.empty())
    {
        return;
    }

    const int maxExtra = mScn.maxgeom - mScn.ngeom;
    if (maxExtra <= 0)
    {
        return;
    }

    const size_t pointCount = points.size();
    size_t step = 1;
    if (static_cast<int>(pointCount) > maxExtra)
    {
        step = static_cast<size_t>((pointCount + maxExtra - 1) / maxExtra);
    }

    const mjtNum mat[9] =
    {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };

    for (size_t i = 0; i < pointCount && mScn.ngeom < mScn.maxgeom; i += step)
    {
        mjvGeom* geom = mScn.geoms + mScn.ngeom++;
        mjtNum size[3] = { 0.006, 0.0, 0.0 };
        mjtNum pos[3] = { static_cast<mjtNum>(points[i][0]),
                          static_cast<mjtNum>(points[i][1]),
                          static_cast<mjtNum>(points[i][2])
                        };
        mjv_initGeom(geom, mjGEOM_SPHERE, size, pos, mat, rgba);
        geom->category = mjCAT_DECOR;
    }
}

void dxMuJoCoRobotViewer::appendFrameGeoms(const std::vector<std::array<double, 12>>& frames)
{
    if (frames.empty())
    {
        return;
    }

    const float axisColors[3][4] =
    {
        { 1.0f, 0.1f, 0.1f, 1.0f },
        { 0.1f, 1.0f, 0.1f, 1.0f },
        { 0.1f, 0.1f, 1.0f, 1.0f }
    };

    const mjtNum size[3] = { 0.05, 0.005, 0.005 };

    for (const auto& frame : frames)
    {
        mjtNum pos[3] =
        {
            static_cast<mjtNum>(frame[0]),
            static_cast<mjtNum>(frame[1]),
            static_cast<mjtNum>(frame[2])
        };

        mjtNum baseMat[9];
        for (int i = 0; i < 9; ++i)
        {
            baseMat[i] = static_cast<mjtNum>(frame[3 + i]);
        }

        const mjtNum* xaxis = baseMat + 0;
        const mjtNum* yaxis = baseMat + 3;
        const mjtNum* zaxis = baseMat + 6;

        const mjtNum mats[3][9] =
        {
            { xaxis[0], xaxis[1], xaxis[2], yaxis[0], yaxis[1], yaxis[2], zaxis[0], zaxis[1], zaxis[2] },
            { yaxis[0], yaxis[1], yaxis[2], zaxis[0], zaxis[1], zaxis[2], xaxis[0], xaxis[1], xaxis[2] },
            { zaxis[0], zaxis[1], zaxis[2], xaxis[0], xaxis[1], xaxis[2], yaxis[0], yaxis[1], yaxis[2] }
        };

        for (int axis = 0; axis < 3 && mScn.ngeom < mScn.maxgeom; ++axis)
        {
            mjvGeom* geom = mScn.geoms + mScn.ngeom++;
            mjv_initGeom(geom, mjGEOM_ARROW, size, pos, mats[axis], axisColors[axis]);
            geom->category = mjCAT_DECOR;
        }
    }
}

void dxMuJoCoRobotViewer::shutdownVisuals()
{
    if (!mVisualsReady)
    {
        return;
    }
    mjr_freeContext(&mCon);
    mjv_freeScene(&mScn);
    mVisualsReady = false;
}

void dxMuJoCoRobotViewer::shutdownModel()
{
    if (mData && mOwnsData)
    {
        mj_deleteData(mData);
    }
    if (mModel && mOwnsModel)
    {
        mj_deleteModel(mModel);
    }
    mData = nullptr;
    mModel = nullptr;
    mOwnsData = false;
    mOwnsModel = false;
    mStreamCamera.setModel(nullptr, nullptr);
}
