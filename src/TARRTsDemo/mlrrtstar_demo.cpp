#include <QApplication>
#include "mlrrtstar_window.h"

using namespace mlrrts;

int main(int argc, char *argv[]) {
    bool no_gui = false;
    QString filename;
    if(argc > 1) {
        no_gui = true;
        filename = QString(argv[1]);
    }
    QApplication a(argc, argv);
    MLRRTstarWindow w;
    if(no_gui) {
        //std::cout << "NO GUI" << std::endl;
        if(w.setupPlanning(filename)) {
            qDebug() << "CONFIG FILE LOADED";
        }
        w.planPath();
        if(w.exportPaths()) {
            qDebug() << "PATH EXPORTED";
        }
        return 0;
    }

    w.show();
    
    return a.exec();
}
