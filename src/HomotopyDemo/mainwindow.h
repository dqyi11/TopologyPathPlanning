#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QPixmap>
#include <QProgressBar>
#include <QMessageBox>
#include "homotopyviz.h"

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

}

#endif // MAINWINDOW_H
