#pragma once

#include <string>

#include <QOpenGLWindow>
#include <mujoco/mujoco.h>

class dxMuJoCoWindow : public QOpenGLWindow
{
    Q_OBJECT

public:
    explicit dxMuJoCoWindow(QWindow* parent = nullptr);
    ~dxMuJoCoWindow() override;

    bool loadModel(const std::string& modelPath);
    void setModel(mjModel* model, mjData* data, bool ownsModelData);
    void reset();

    mjModel* model() const { return mModel; }
    mjData* data() const { return mData; }

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
    void shutdownVisuals();
    void shutdownModel();

    std::string mModelPath;
    mjModel* mModel = nullptr;
    mjData* mData = nullptr;
    bool mOwnsModelData = false;

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
};
