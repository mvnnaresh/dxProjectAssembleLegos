#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDockWidget>
#include <QMetaObject>
#include <QMetaType>
#include <QPushButton>
#include <QVBoxLayout>
#include <QString>

#include <array>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("DexMan Digital Twin + Viewer");

    qRegisterMetaType<std::vector<double>>("std::vector<double>");
    qRegisterMetaType<std::vector<std::array<double, 3>>>("std::vector<std::array<double, 3>>");

    mViewer = new dxMuJoCoRobotViewer();
    QWidget* container = QWidget::createWindowContainer(mViewer, this);
    setCentralWidget(container);

    QDockWidget* dock = new QDockWidget("Controls", this);
    QWidget* dockWidget = new QWidget(dock);
    QVBoxLayout* dockLayout = new QVBoxLayout(dockWidget);
    QPushButton* initButton = new QPushButton("Init", dockWidget);
    QPushButton* testButton = new QPushButton("Test Planner", dockWidget);
    QPushButton* cartButton = new QPushButton("Test Cartesian", dockWidget);
    QPushButton* pickPlaceButton = new QPushButton("Test Pick/Place", dockWidget);
    QPushButton* closeButton = new QPushButton("Close Gripper", dockWidget);
    QPushButton* openButton = new QPushButton("Open Gripper", dockWidget);
    QSlider* gripperSlider = new QSlider(Qt::Horizontal, dockWidget);
    gripperSlider->setRange(0, 100);
    gripperSlider->setValue(0);
    dockLayout->addWidget(initButton);
    dockLayout->addWidget(testButton);
    dockLayout->addWidget(cartButton);
    dockLayout->addWidget(pickPlaceButton);
    dockLayout->addWidget(closeButton);
    dockLayout->addWidget(openButton);
    dockLayout->addWidget(gripperSlider);
    dockLayout->addStretch(1);
    dockWidget->setLayout(dockLayout);
    dock->setWidget(dockWidget);
    addDockWidget(Qt::LeftDockWidgetArea, dock);

    mSim = new dxMuJoCoRobotSimulator();
    mSim->setControlRateHz(250.0);
    mSimThread = new QThread(this);
    mSim->moveToThread(mSimThread);
    connect(mSimThread, &QThread::finished, mSim, &QObject::deleteLater);
    mSimThread->start();

    connect(mSim, &dxMuJoCoRobotSimulator::modelLoaded, this, &MainWindow::onModelLoaded);
    connect(mSim, &dxMuJoCoRobotSimulator::stateUpdated, this, &MainWindow::onStateUpdated);
    connect(initButton, &QPushButton::clicked, this, [this]()
    {
        if (!mDemo)
        {
            mDemo = std::make_unique<demo>(mSim);
            connect(mDemo.get(), &demo::ctrlTargetsReady,
                    mSim, &dxMuJoCoRobotSimulator::setCtrlTargets, Qt::QueuedConnection);
            connect(mDemo.get(), &demo::jointPositionsReady,
                    mSim, &dxMuJoCoRobotSimulator::setJointPositions, Qt::QueuedConnection);
            connect(mDemo.get(), &demo::ctrlTargetsFromJointsReady,
                    mSim, &dxMuJoCoRobotSimulator::setCtrlTargetsFromJointPositions, Qt::QueuedConnection);
            connect(mDemo.get(), &demo::drawTrajectory,
                    mViewer, &dxMuJoCoRobotViewer::drawTrajectory);
            connect(mDemo.get(), &demo::drawFrames,
                    mViewer, &dxMuJoCoRobotViewer::drawFrames);
            connect(mDemo.get(), &demo::closeGripperRequested,
                    mSim, &dxMuJoCoRobotSimulator::closeGripper, Qt::QueuedConnection);
            connect(mDemo.get(), &demo::gripperPositionRequested,
                    mSim, &dxMuJoCoRobotSimulator::setGripperPosition, Qt::QueuedConnection);
        }
        if (mViewer)
        {
            mViewer->setModel(nullptr);
        }
        QMetaObject::invokeMethod(mSim, "loadModel", Qt::QueuedConnection,
                                  Q_ARG(QString, QString::fromStdString(mModelPath)));

    });

    connect(testButton, &QPushButton::clicked, this, [this]()
    {
        if (mDemo)
        {
            mDemo->testPlannerSimple();
        }
    });

    connect(cartButton, &QPushButton::clicked, this, [this]()
    {
        if (mDemo)
        {
            mDemo->testPlannerCartesian();
        }
    });
    connect(pickPlaceButton, &QPushButton::clicked, this, [this]()
    {
        if (mDemo)
        {
            mDemo->testPickAndPlace();
        }
    });

    connect(closeButton, &QPushButton::clicked, this, [this]()
    {
        if (mDemo)
        {
            mDemo->closeGripper();
        }
    });
    connect(openButton, &QPushButton::clicked, this, [this]()
    {
        if (mDemo)
        {
            mDemo->openGripper();
        }
    });
    connect(gripperSlider, &QSlider::valueChanged, this, [this](int value)
    {
        if (mDemo)
        {
            mDemo->setGripperPosition(static_cast<double>(value) / 100.0);
        }
    });

}

MainWindow::~MainWindow()
{
    if (mSim)
    {
        QMetaObject::invokeMethod(mSim, "stop", Qt::QueuedConnection);
    }
    if (mSimThread)
    {
        mSimThread->quit();
        mSimThread->wait();
    }
    delete ui;
}

void MainWindow::onModelLoaded(mjModel* model)
{
    if (mViewer)
    {
        mViewer->setModel(model);
        if (mSim)
        {
            mViewer->applyState(mSim->getRobotState());
        }
    }
    if (mDemo && !mDemo->init())
    {
        return;
    }
    if (mSim)
    {
        QMetaObject::invokeMethod(mSim, "start", Qt::QueuedConnection);
    }
}

void MainWindow::onStateUpdated()
{
    if (mViewer && mSim)
    {
        mViewer->applyState(mSim->getRobotState());
    }
}
