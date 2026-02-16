#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include <memory>

#include "dxMuJoCoWindow.h"
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

private:
    Ui::MainWindow *ui;
    dxMuJoCoWindow* mViewer = nullptr;
    QTimer* mSimTimer = nullptr;
    std::unique_ptr<demo> mDemo;
    std::string mModelPath = "models/ur10e_2f85_scene.xml";
};

#endif // MAINWINDOW_H
