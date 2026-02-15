#include "mainwindow.h"

#include <QApplication>
#include <QSurfaceFormat>

int main(int argc, char *argv[])
{
    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    fmt.setSamples(0);
    fmt.setVersion(2, 1);
    fmt.setProfile(QSurfaceFormat::CompatibilityProfile);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication::setAttribute(Qt::AA_DisableHighDpiScaling);

    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowFlags(Qt::WindowTitleHint | Qt::WindowMinimizeButtonHint
                     | Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint | Qt::WindowSystemMenuHint);
    w.show();

    return a.exec();
}




// #include <string>
// #include "demo.h"
//
// int main()
// {
//     const std::string modelPath = "models/ur10e_2f85_scene.xml";
//     demo app(modelPath, true);
//     if (!app.init())
//         return -1;
//
//     app.testPlanner();
//
//     // app.test();
//     app.run(10);
//     return 0;
// }
