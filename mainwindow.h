#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>

#include <memory>

#include "dxMuJoCoRobotSimulator.h"
#include "dxMuJoCoRobotViewer.h"
#include "demo.h"

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

private slots:
    void onModelLoaded(mjModel* model);
    void onStateUpdated();

private:
    Ui::MainWindow *ui;
    dxMuJoCoRobotViewer* mViewer = nullptr;
    dxMuJoCoRobotSimulator* mSim = nullptr;
    QThread* mSimThread = nullptr;
    std::unique_ptr<demo> mDemo;
    std::string mModelPath = "models/ur10e_2f85_scene.xml";
};

#endif // MAINWINDOW_H
