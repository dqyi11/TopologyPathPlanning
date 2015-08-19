#include "mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QtDebug>
#include <QKeyEvent>
#include <QStatusBar>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent) {
    mpViz = new HomotopyViz();

    createActions();
    createMenuBar();

    setCentralWidget(mpViz);
}

MainWindow::~MainWindow() {

    if(mpViz) {
        delete mpViz;
        mpViz = NULL;
    }
}

void MainWindow::createMenuBar() {
    mpFileMenu = menuBar()->addMenu("&File");
    mpFileMenu->addAction(mpOpenAction);
}

void MainWindow::createActions() {
    mpOpenAction = new QAction("Open", this);
    connect(mpOpenAction, SIGNAL(triggered()), this, SLOT(onOpen()));
}

void MainWindow::onOpen() {
    QString tempFilename = QFileDialog::getOpenFileName(this,
             tr("Open File"), "./", tr("Map Files (*.*)"));

    mpViz->loadMap(tempFilename);
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
   if (event->key() == Qt::Key_R  ) {
       if(mpViz) {
           if(mpViz->mShowSubregion==true) {
               mpViz->mShowSubregion = false;
           }
           else {
               mpViz->mShowSubregion = true;
           }
           repaint();
       }
   }
   else if(event->key() == Qt::Key_Up ) {
       if(mpViz) {
           mpViz->nextRegion();
           repaint();
       }
   }
   else if(event->key() == Qt::Key_Down ) {
       if(mpViz) {
           mpViz->prevRegion();
           repaint();
       }
   }
}
