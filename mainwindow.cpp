#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDockWidget>
#include <QMetaObject>
#include <QMetaType>
#include <QPushButton>
#include <QVBoxLayout>
#include <QString>
#include <QSlider>
#include <QPixmap>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

#include <array>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("DexMan Digital Twin + Viewer");

    qRegisterMetaType<std::vector<double>>("std::vector<double>");
    qRegisterMetaType<std::vector<std::array<double, 3>>>("std::vector<std::array<double, 3>>");

    mViewer = new dxMuJoCoRobotViewer();
    mViewerContainer = QWidget::createWindowContainer(mViewer, this);
    setCentralWidget(mViewerContainer);

    QDockWidget* dock = new QDockWidget("Controls", this);
    QWidget* dockWidget = new QWidget(dock);
    QVBoxLayout* dockLayout = new QVBoxLayout(dockWidget);
    QPushButton* initButton = new QPushButton("Init", dockWidget);
    QPushButton* testButton = new QPushButton("Test Planner", dockWidget);
    QPushButton* cartButton = new QPushButton("Test Cartesian", dockWidget);
    QPushButton* pickPlaceButton = new QPushButton("Test Pick/Place", dockWidget);
    mCameraButton = new QPushButton("Test Camera", dockWidget);
    QPushButton* closeButton = new QPushButton("Close Gripper", dockWidget);
    QPushButton* openButton = new QPushButton("Open Gripper", dockWidget);
    QSlider* gripperSlider = new QSlider(Qt::Horizontal, dockWidget);

    gripperSlider->setRange(0, 100);
    gripperSlider->setValue(0);
    dockLayout->addWidget(initButton);
    dockLayout->addWidget(testButton);
    dockLayout->addWidget(cartButton);
    dockLayout->addWidget(pickPlaceButton);
    dockLayout->addWidget(mCameraButton);
    dockLayout->addWidget(closeButton);
    dockLayout->addWidget(openButton);
    dockLayout->addWidget(gripperSlider);
    dockLayout->addStretch(1);
    dockWidget->setLayout(dockLayout);
    dock->setWidget(dockWidget);
    addDockWidget(Qt::LeftDockWidgetArea, dock);

    mCameraLabel = new QLabel(mViewerContainer);
    mCameraLabel->setFixedSize(320, 240);
    mCameraLabel->setAlignment(Qt::AlignCenter);
    mCameraLabel->setStyleSheet("background-color: rgba(10, 10, 10, 180);");
    mCameraLabel->move(10, 10);
    mCameraLabel->hide();
    updateCameraButtonState();

    connect(mViewer, &dxMuJoCoRobotViewer::rgbFrameReady, this, [this](const QImage& image)
    {
        if (!mCameraLabel)
        {
            return;
        }
        QPixmap pix = QPixmap::fromImage(image);
        mCameraLabel->setPixmap(pix.scaled(mCameraLabel->size(),
                                           Qt::KeepAspectRatio,
                                           Qt::SmoothTransformation));
    });
    mSim = new dxMuJoCoRobotSimulator();
    mSim->setControlRateHz(250.0);
    mSimThread = new QThread(this);
    mSim->moveToThread(mSimThread);

    auto resolveModelPath = [](const std::string& relPath) -> std::string
    {
        const QString path = QString::fromStdString(relPath);
        QFileInfo info(path);
        if (info.exists())
        {
            return info.absoluteFilePath().toStdString();
        }

        const QString base = QCoreApplication::applicationDirPath();
        const QStringList candidates =
        {
            QDir(base).filePath(path),
            QDir(base).filePath(QString("../%1").arg(path)),
            QDir(base).filePath(QString("../../%1").arg(path))
        };

        for (const QString& candidate : candidates)
        {
            QFileInfo candidateInfo(candidate);
            if (candidateInfo.exists())
            {
                return candidateInfo.absoluteFilePath().toStdString();
            }
        }

        return relPath;
    };

    mModelPath = resolveModelPath(mModelPath);

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
            connect(mDemo.get(), &demo::updateJointConfig,
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

            connect(mDemo.get(), &demo::updateUIMessage,
                    this, &MainWindow::setStatusMessage);
            connect(mDemo.get(), &demo::cameraStreamRequested,
                    mViewer, &dxMuJoCoRobotViewer::setCameraStreamEnabled);
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

    connect(mCameraButton, &QPushButton::clicked, this, [this]()
    {
        mCameraStreaming = !mCameraStreaming;
        if (mViewer)
        {
            mViewer->setCameraStreamEnabled(mCameraStreaming);
        }
        if (mCameraLabel)
        {
            mCameraLabel->setVisible(mCameraStreaming);
            if (mCameraStreaming)
            {
                mCameraLabel->raise();
            }
        }
        updateCameraButtonState();
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

void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    if (mCameraLabel && mViewerContainer)
    {
        mCameraLabel->move(10, 10);
        mCameraLabel->raise();
    }
}

void MainWindow::onModelLoaded(mjModel* model) const
{
    if (mViewer)
    {
        mViewer->setModel(model);
        mViewer->setCameraStreamName("scene_cam");
        mViewer->setCameraStreamResolution(640, 480);
        this->onStateUpdated();
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

void MainWindow::onStateUpdated() const
{
    if (mViewer && mSim)
    {
        mViewer->applyState(mSim->getRobotState());
    }
}

void MainWindow::setStatusMessage(const std::string& msg) const
{
    ui->statusBar->showMessage(QString::fromStdString(msg));
}

void MainWindow::updateCameraButtonState()
{
    if (!mCameraButton)
    {
        return;
    }
    if (mCameraStreaming)
    {
        mCameraButton->setStyleSheet("background-color: #1f8f4a; color: white;");
    }
    else
    {
        mCameraButton->setStyleSheet("background-color: #b23b3b; color: white;");
    }
}
