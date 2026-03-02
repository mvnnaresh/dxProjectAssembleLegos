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

    mInterface = new dxMujocoInterface(this);
    mViewer = mInterface->viewer();
    mViewerContainer = QWidget::createWindowContainer(mViewer, this);
    setCentralWidget(mViewerContainer);

    // Cleanup happens in closeEvent for a graceful shutdown.


    QDockWidget* dock = new QDockWidget("Controls", this);
    QWidget* dockWidget = new QWidget(dock);
    QVBoxLayout* dockLayout = new QVBoxLayout(dockWidget);
    QPushButton* initButton = new QPushButton("Init", dockWidget);
    QPushButton* testButton = new QPushButton("Test Planner", dockWidget);
    QPushButton* cartButton = new QPushButton("Test Cartesian", dockWidget);
    QPushButton* pickPlaceButton = new QPushButton("Test Pick/Place", dockWidget);
    QPushButton* pickPlaceFullButton = new QPushButton("Test Pick/Place Full", dockWidget);
    QPushButton* assembleButton = new QPushButton("Test Lego Assembly", dockWidget);
    mCameraButton = new QPushButton("Test Camera", dockWidget);
    QPushButton* camera3dButton = new QPushButton("Test Camera 3D", dockWidget);
    QPushButton* closeButton = new QPushButton("Close Gripper", dockWidget);
    QPushButton* openButton = new QPushButton("Open Gripper", dockWidget);
    QPushButton* quitButton = new QPushButton("Quit", dockWidget);
    QSlider* gripperSlider = new QSlider(Qt::Horizontal, dockWidget);

    gripperSlider->setRange(0, 100);
    gripperSlider->setValue(0);
    dockLayout->addWidget(initButton);
    dockLayout->addWidget(testButton);
    dockLayout->addWidget(cartButton);
    dockLayout->addWidget(pickPlaceButton);
    dockLayout->addWidget(pickPlaceFullButton);
    dockLayout->addWidget(assembleButton);
    dockLayout->addWidget(mCameraButton);
    dockLayout->addWidget(camera3dButton);
    dockLayout->addWidget(closeButton);
    dockLayout->addWidget(openButton);
    dockLayout->addWidget(gripperSlider);
    dockLayout->addWidget(quitButton);
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

    connect(mViewer, &dxMuJoCoRobotViewer::pointCloudReady,
            this,
            [this](const std::vector<std::array<float, 3>>& points,
                   const std::vector<std::array<unsigned char, 3>>& colors)
    {
        if (points.empty() || colors.empty() || points.size() != colors.size())
        {
            return;
        }
        CloudPtr cloud(new Cloud());
        cloud->points.resize(points.size());
        cloud->width = static_cast<uint32_t>(points.size());
        cloud->height = 1;
        cloud->is_dense = false;
        for (size_t i = 0; i < points.size(); ++i)
        {
            const auto& p = points[i];
            const auto& c = colors[i];
            PointRGBA pt;
            pt.x = p[0];
            pt.y = p[1];
            pt.z = p[2];
            pt.r = c[0];
            pt.g = c[1];
            pt.b = c[2];
            pt.a = 255;
            cloud->points[i] = pt;
        }
        cropvalues limits;
        limits.xmin = 0.0;
        limits.xmax = 1.0;
        limits.ymin = -1.0;
        limits.ymax = 1.0;
        limits.zmin = 0.0;
        limits.zmax = 0.8;

        auto& vision = mVisionByCamera["scene_cam"];
        if (!vision)
        {
            vision = std::make_unique<dxVision>();
        }
        CloudPtr cropped = vision->cropPointCloud(cloud, limits);
        if (cropped && !cropped->points.empty())
        {
            vision->viewPointCloud(cropped, "camera_cloud", "Camera Point Cloud (Cropped)", 2);
        }
        else
        {
            vision->viewPointCloud(cloud, "camera_cloud_raw", "Camera Point Cloud (Raw)", 2);
        }
    });

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

    connect(mInterface, &dxMujocoInterface::modelLoaded, this, &MainWindow::onModelLoaded);
    connect(mInterface, &dxMujocoInterface::stateUpdated, this, &MainWindow::onStateUpdated);

    connect(initButton, &QPushButton::clicked, this, [this]()
    {
        if (!mDemo)
        {
            mDemo = std::make_unique<demo>(mInterface);
            connect(mDemo.get(), &demo::drawTrajectory,
                    mViewer, &dxMuJoCoRobotViewer::drawTrajectory);
            connect(mDemo.get(), &demo::drawFrames,
                    mViewer, &dxMuJoCoRobotViewer::drawFrames);
            connect(mDemo.get(), &demo::updateUIMessage,
                    this, &MainWindow::setStatusMessage);
        }
        if (mViewer)
        {
            mViewer->setModel(nullptr);
        }
        mInterface->loadModel(QString::fromStdString(mModelPath));

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
    connect(pickPlaceFullButton, &QPushButton::clicked, this, [this]()
    {
        if (mDemo)
        {
            mDemo->testNewPickAndPlace();
        }
    });
    connect(assembleButton, &QPushButton::clicked, this, [this]()
    {
        if (mDemo)
        {
            mDemo->testLegoAssembly();
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

    connect(camera3dButton, &QPushButton::clicked, this, [this]()
    {
        if (mDemo)
        {
            mDemo->testCamera3D();
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

    connect(quitButton, &QPushButton::clicked, this, [this]()
    {
        close();
    });


}

MainWindow::~MainWindow()
{
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
        mViewer->setCameraStreamBaseBodyName("base");
        this->onStateUpdated();
    }
    if (mDemo && !mDemo->init())
    {
        return;
    }
    if (mInterface)
    {
        mInterface->start();
    }
}

void MainWindow::onStateUpdated() const
{
    if (mViewer && mInterface)
    {
        mViewer->applyState(mInterface->getRobotState());
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

void MainWindow::shutdownApp()
{
    if (mViewer)
    {
        mViewer->setModel(nullptr);
    }
    if (mInterface)
    {
        mInterface->clearViewer();
        mInterface->shutdown();
    }
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    shutdownApp();
    QMainWindow::closeEvent(event);
}
