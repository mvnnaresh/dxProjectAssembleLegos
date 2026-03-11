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

namespace
{
    constexpr double kTaskSpaceFovyDeg = 25.0;
}

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


    mControlDock = new QDockWidget("Controls", this);
    mControlDock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
    mControlScroll = new QScrollArea(mControlDock);
    mControlScroll->setWidgetResizable(true);
    mControlWidget = new QWidget(mControlScroll);
    QVBoxLayout* dockLayout = new QVBoxLayout(mControlWidget);
    dockLayout->setContentsMargins(16, 16, 16, 16);
    dockLayout->setSpacing(12);
    auto makeSection = [&](const QString& text)
    {
        QLabel* label = new QLabel(text, mControlWidget);
        label->setObjectName("sectionLabel");
        mSectionLabels.push_back(label);
        return label;
    };

    QPushButton* initButton = new QPushButton("Init", mControlWidget);
    QPushButton* testButton = new QPushButton("Test Planner", mControlWidget);
    QPushButton* cartButton = new QPushButton("Test Cartesian", mControlWidget);
    QPushButton* pickPlaceButton = new QPushButton("Test Pick/Place", mControlWidget);
    QPushButton* pickPlaceFullButton = new QPushButton("Test Pick/Place Full", mControlWidget);
    QPushButton* assembleButton = new QPushButton("Test Lego Assembly", mControlWidget);
    mCameraButton = new QPushButton("Test Camera", mControlWidget);
    QPushButton* camera3dButton = new QPushButton("Test Camera 3D", mControlWidget);
    QPushButton* closeButton = new QPushButton("Close Gripper", mControlWidget);
    QPushButton* openButton = new QPushButton("Open Gripper", mControlWidget);
    QPushButton* quitButton = new QPushButton("Quit", mControlWidget);
    mGripperSlider = new QSlider(Qt::Horizontal, mControlWidget);

    mGripperSlider->setRange(0, 100);
    mGripperSlider->setValue(0);
    mDockButtons = { initButton, testButton, cartButton, pickPlaceButton,
                     pickPlaceFullButton, assembleButton, mCameraButton,
                     camera3dButton, closeButton, openButton, quitButton
                   };
    for (QPushButton* button : mDockButtons)
    {
        button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        button->setMinimumHeight(42);
    }
    mGripperSlider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    mGripperSlider->setMinimumHeight(26);

    dockLayout->addWidget(makeSection("Simulation"));
    dockLayout->addWidget(initButton);
    dockLayout->addSpacing(12);

    dockLayout->addSpacing(8);
    dockLayout->addWidget(makeSection("Planning"));
    dockLayout->addWidget(testButton);
    dockLayout->addWidget(cartButton);
    dockLayout->addWidget(pickPlaceButton);
    dockLayout->addWidget(pickPlaceFullButton);
    dockLayout->addWidget(assembleButton);
    dockLayout->addSpacing(12);

    dockLayout->addSpacing(8);
    dockLayout->addWidget(makeSection("Vision"));
    dockLayout->addWidget(mCameraButton);
    dockLayout->addWidget(camera3dButton);
    dockLayout->addSpacing(12);

    dockLayout->addSpacing(8);
    dockLayout->addWidget(makeSection("Gripper"));
    dockLayout->addWidget(closeButton);
    dockLayout->addWidget(openButton);
    dockLayout->addWidget(mGripperSlider);
    dockLayout->addSpacing(12);

    dockLayout->addStretch(1);
    dockLayout->addWidget(quitButton);
    mControlWidget->setLayout(dockLayout);
    mControlWidget->setStyleSheet(
        "QWidget {"
        "  background-color: #111418;"
        "  color: #e7edf6;"
        "  font-family: \"Segoe UI Variable\";"
        "  font-size: 28px;"
        "}"
        "QLabel#sectionLabel {"
        "  color: #FF7F50;"
        "  font-size: 28px;"
        "  font-weight: 600;"
        "  padding: 8px 4px 2px 4px;"
        "  text-transform: uppercase;"
        "}"
        "QPushButton {"
        "  background-color: #1b222b;"
        "  border: 1px solid #2b3440;"
        "  border-radius: 10px;"
        "  padding: 10px 14px;"
        "  text-align: center;"
        "  font-weight: 600;"
        "}"
        "QPushButton:hover {"
        "  background-color: #243040;"
        "  border-color: #3c4a5b;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #161d25;"
        "}"
        "QPushButton:disabled {"
        "  color: #8b98a7;"
        "  background-color: #202933;"
        "  border-color: #2a3440;"
        "}"
        "QSlider::groove:horizontal {"
        "  height: 8px;"
        "  background: #1b222b;"
        "  border-radius: 4px;"
        "}"
        "QSlider::handle:horizontal {"
        "  width: 18px;"
        "  margin: -6px 0;"
        "  border-radius: 9px;"
        "  background: #7ac5ff;"
        "  border: 1px solid #5aa8e6;"
        "}"
        "QSlider::sub-page:horizontal {"
        "  background: #3b82f6;"
        "  border-radius: 4px;"
        "}"
    );
    applyControlStyling();
    updateControlSizing();
    mControlScroll->setWidget(mControlWidget);
    mControlDock->setWidget(mControlScroll);
    mControlDock->setMinimumWidth(280);
    addDockWidget(Qt::LeftDockWidgetArea, mControlDock);

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
    connect(mGripperSlider, &QSlider::valueChanged, this, [this](int value)
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
    updateControlSizing();
}

void MainWindow::onModelLoaded(mjModel* model) const
{
    if (mViewer)
    {
        mViewer->setModel(model);
        mViewer->setCameraStreamName("scene_cam");
        mViewer->setCameraStreamResolution(640, 480);
        mViewer->setCameraStreamFovy(kTaskSpaceFovyDeg);
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

void MainWindow::applyControlStyling()
{
    QFont font("Segoe UI");
    font.setPointSize(14);
    mControlWidget->setFont(font);
    for (QPushButton* button : mDockButtons)
    {
        if (!button)
        {
            continue;
        }
        button->setFont(font);
    }
    if (mGripperSlider)
    {
        mGripperSlider->setFont(font);
    }
}

void MainWindow::updateControlSizing()
{
    if (!mControlWidget || mDockButtons.isEmpty())
    {
        return;
    }

    const int availableHeight = mControlWidget->height();
    const int buttonCount = mDockButtons.size();
    const int baseHeight = (buttonCount > 0) ? (availableHeight / (buttonCount + 6)) : 44;
    const int buttonHeight = std::max(40, std::min(64, baseHeight));
    const int fontSize = std::max(12, std::min(16, buttonHeight / 2));
    const int sectionFontSize = std::max(11, std::min(13, fontSize - 2));
    const int sliderHeight = std::max(24, buttonHeight / 2);

    QFont buttonFont("Segoe UI Variable");
    buttonFont.setPointSize(fontSize);
    QFont sectionFont("Segoe UI Variable");
    sectionFont.setPointSize(sectionFontSize);

    for (QPushButton* button : mDockButtons)
    {
        if (!button)
        {
            continue;
        }
        button->setFont(buttonFont);
        button->setMinimumHeight(buttonHeight);
        button->setMaximumHeight(buttonHeight);
    }

    for (QLabel* label : mSectionLabels)
    {
        if (!label)
        {
            continue;
        }
        label->setFont(sectionFont);
    }

    if (mGripperSlider)
    {
        mGripperSlider->setFont(buttonFont);
        mGripperSlider->setMinimumHeight(sliderHeight);
        mGripperSlider->setMaximumHeight(sliderHeight);
    }
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    shutdownApp();
    QMainWindow::closeEvent(event);
}
