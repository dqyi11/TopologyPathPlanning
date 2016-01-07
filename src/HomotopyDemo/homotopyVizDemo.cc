//#include <QtGui/QApplication>
#include <QApplication>
#include "mainwindow.h"

using namespace homotopy;

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
    
  return a.exec();
}
