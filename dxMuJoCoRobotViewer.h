#pragma once

#include <array>
#include <functional>
#include <string>
#include <vector>

#include <QImage>
#include <QOpenGLWindow>
#include <QString>
#include <mujoco/mujoco.h>

#include "dxMuJoCoRobotState.h"
#include "dxMuJoCoRealSense.h"

class dxMuJoCoRobotViewer : public QOpenGLWindow
{
    Q_OBJECT

public:
    explicit dxMuJoCoRobotViewer(QWindow* parent = nullptr);
    ~dxMuJoCoRobotViewer() override;

    bool loadModel(const std::string& modelPath);
    void setModel(mjModel* model);
    void reset();

    void drawTrajectory(const std::vector<std::array<double, 3>>& points);
    void drawFrames(const std::vector<std::array<double, 12>>& frames);
    void runWithContext(const std::function<void()>& fn);

    mjModel* model() const
    {
        return mModel;
    }
    mjData* data() const
    {
        return mData;
    }
    mjvOption* mjvOptionPtr()
    {
        return mVisualsReady ? &mOpt : nullptr;
    }
    mjrContext* mjrContextPtr()
    {
        return mVisualsReady ? &mCon : nullptr;
    }

public slots:
    void applyState(const dxMuJoCoRobotState& state);
    void setCameraStreamEnabled(bool enabled);
    void setCameraStreamName(const QString& name);
    void setCameraStreamResolution(int width, int height);
    void setCameraStreamFovy(double fovyDeg);
    void setCameraStreamBaseBodyName(const QString& name);
    void requestPointCloudCapture();

signals:
    void rgbFrameReady(const QImage& image);
    void pointCloudReady(const std::vector<std::array<float, 3>>& points,
                         const std::vector<std::array<unsigned char, 3>>& colors);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void initVisuals();
    void appendPathGeoms(const std::vector<std::array<double, 3>>& points, const float rgba[4]);
    void appendFrameGeoms(const std::vector<std::array<double, 12>>& frames);
    void shutdownVisuals();
    void shutdownModel();

    std::string mModelPath;
    mjModel* mModel = nullptr;
    mjData* mData = nullptr;
    bool mOwnsModel = false;
    bool mOwnsData = false;

    mjvCamera mCam;
    mjvOption mOpt;
    mjvScene mScn;
    mjrContext mCon;
    bool mVisualsReady = false;
    bool mNeedsVisualInit = false;

    bool mBtnLeft = false;
    bool mBtnMiddle = false;
    bool mBtnRight = false;
    double mLastX = 0.0;
    double mLastY = 0.0;

    std::vector<std::array<double, 3>> mTrajectoryPath;
    std::vector<std::array<double, 12>> mDebugFrames;
    size_t mMaxPathPoints = 4000;

    bool mStreamEnabled = false;
    int mStreamWidth = 640;
    int mStreamHeight = 480;
    QString mStreamCameraName;
    QString mStreamBaseBodyName;
    dxMuJoCoRealSense mStreamCamera;
    bool mStreamErrorLogged = false;
    bool mPointCloudCapturePending = false;
    bool mPointCloudErrorLogged = false;
    std::function<void()> mPendingContextFn;
};
