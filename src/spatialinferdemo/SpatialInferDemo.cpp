//#include <QtGui/QApplication>
#include <QApplication>
#include "SpatialInferWindow.hpp"

using namespace topologyPathPlanning::topologyinference;

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  SpatialInferWindow w;
  w.show();
    
  return a.exec();
}
