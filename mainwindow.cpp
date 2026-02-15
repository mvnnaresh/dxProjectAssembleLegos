#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    mViewer = new dxMuJoCoWindow();
    QWidget* container = QWidget::createWindowContainer(mViewer, this);
    setCentralWidget(container);
    mViewer->loadModel("models/ur10e_2f85_scene.xml");
}

MainWindow::~MainWindow()
{
    delete ui;
}
