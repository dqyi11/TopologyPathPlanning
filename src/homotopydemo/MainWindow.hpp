#ifndef TOPOLOGYPATPATHPLANNING_HOMOTOPY_MAINWINDOW_HPP
#define TOPOLOGYPATPATHPLANNING_HOMOTOPY_MAINWINDOW_HPP

#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QPixmap>
#include <QProgressBar>
#include <QMessageBox>

#include "topologyPathPlanning/homotopyviz/HomotopyViz.hpp"

namespace topologyPathPlanning {

namespace homotopy {

  class MainWindow : public QMainWindow {
    Q_OBJECT
    
  public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    HomotopyViz * mpViz;

  protected:
    void createMenuBar();
    void createActions();
    void updateStatusBar();

    void keyPressEvent(QKeyEvent * e);
  private:
    QMessageBox*  mpMsgBox;
    QMenu*        mpFileMenu;
    QAction*      mpOpenAction;
    QAction*      mpSaveAction;
    QAction*      mpLoadAction;
    QLabel*       mpStatusLabel;

  private slots:
    void onOpen();
    void onSave();
    void onLoad();
  };

} // homotopy

} // topologyPathPlanning

#endif // TOPOLOGYPATPATHPLANNING_HOMOTOPY_MAINWINDOW_HPP
