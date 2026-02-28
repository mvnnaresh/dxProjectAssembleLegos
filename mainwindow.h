#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QLabel>
#include <QPushButton>
#include <QDebug>
#include <memory>

#include "dxMuJoCoRobotSimulator.h"
#include "dxMuJoCoRobotViewer.h"
#include "demo.h"
#include "dxVision.h"

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void resizeEvent(QResizeEvent* event) override;


private slots:
    void onModelLoaded(mjModel* model) const;
    void onStateUpdated() const;

private:
    Ui::MainWindow *ui;
    dxMuJoCoRobotViewer* mViewer = nullptr;
    dxMuJoCoRobotSimulator* mSim = nullptr;
    QThread* mSimThread = nullptr;
    std::unique_ptr<demo> mDemo;
    QWidget* mViewerContainer = nullptr;
    QLabel* mCameraLabel = nullptr;
    QPushButton* mCameraButton = nullptr;
    bool mCameraStreaming = false;
    std::unique_ptr<dxVision> mVision;
    std::string mModelPath = "models/ur10e_hande_workbench_scene.xml";

    void setStatusMessage(const std::string& msg) const;
    void updateCameraButtonState();
};

#endif // MAINWINDOW_H
