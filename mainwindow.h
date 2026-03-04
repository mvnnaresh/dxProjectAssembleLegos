#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QDockWidget>
#include <QList>
#include <QScrollArea>
#include <QDebug>
#include <memory>
#include <unordered_map>

#include "dxMuJoCoRobotViewer.h"
#include "dxMujocoInterface.h"
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
    void closeEvent(QCloseEvent* event) override;


private slots:
    void onModelLoaded(mjModel* model) const;
    void onStateUpdated() const;

private:
    Ui::MainWindow *ui;
    dxMuJoCoRobotViewer* mViewer = nullptr;
    dxMujocoInterface* mInterface = nullptr;
    std::unique_ptr<demo> mDemo;
    QWidget* mViewerContainer = nullptr;
    QLabel* mCameraLabel = nullptr;
    QPushButton* mCameraButton = nullptr;
    QDockWidget* mControlDock = nullptr;
    QScrollArea* mControlScroll = nullptr;
    QWidget* mControlWidget = nullptr;
    QSlider* mGripperSlider = nullptr;
    QList<QPushButton*> mDockButtons;
    QList<QLabel*> mSectionLabels;
    bool mCameraStreaming = false;
    std::unordered_map<std::string, std::unique_ptr<dxVision>> mVisionByCamera;
    std::string mModelPath = "models/ur10e_hande_workbench_scene_lego_2x2.xml";

    void setStatusMessage(const std::string& msg) const;
    void updateCameraButtonState();
    void shutdownApp();
    void applyControlStyling();
    void updateControlSizing();
};

#endif // MAINWINDOW_H
