//#include <QtGui/QApplication>
#include <QApplication>
#include "spatialinfer_window.h"

using namespace topology_inference;

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  SpatialInferWindow w;
  w.show();
    
  return a.exec();
}
