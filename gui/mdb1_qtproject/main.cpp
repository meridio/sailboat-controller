#include <QtGui/QApplication>
#include "qmlapplicationviewer.h"

// fileIO includes
#include <QtDeclarative>
#include "fileio.h"

Q_DECL_EXPORT int main(int argc, char *argv[])
{
    QScopedPointer<QApplication> app(createApplication(argc, argv));

    qmlRegisterType<FileIO, 1>("FileIO", 1, 0, "FileIO");

    QmlApplicationViewer viewer;
    viewer.setOrientation(QmlApplicationViewer::ScreenOrientationAuto);
    viewer.setMainQmlFile(QLatin1String("qml/mdb1_qtproject/main.qml"));
    viewer.showExpanded();

    return app->exec();
}
