#pragma once

#include <array>
#include <string>
#include <vector>

#include <QOpenGLWindow>
#include <mujoco/mujoco.h>

#include "dxMuJoCoRobotState.h"

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

    mjModel* model() const
    {
        return mModel;
    }
    mjData* data() const
    {
        return mData;
    }

public slots:
    void applyState(const dxMuJoCoRobotState& state);

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
    size_t mMaxPathPoints = 4000;
};
