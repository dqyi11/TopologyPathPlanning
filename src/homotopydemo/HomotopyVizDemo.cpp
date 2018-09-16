#include <QApplication>
#include "MainWindow.hpp"

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  topologyPathPlanning::homotopy::MainWindow w;
  w.show();
    
  return a.exec();
}
