#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QPixmap>
#include <QProgressBar>
#include "homotopyviz.h"


class MainWindow : public QMainWindow {
    Q_OBJECT
    
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    HomotopyViz * mpViz;

protected:
    void createMenuBar();
    void createActions();

    void keyPressEvent(QKeyEvent * e);
private:
    QMenu*        mpFileMenu;
    QAction*      mpOpenAction;

private slots:
    void onOpen();
};

#endif // MAINWINDOW_H
