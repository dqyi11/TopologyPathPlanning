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
    mpFileMenu->addAction(mpSaveAction);
    mpFileMenu->addAction(mpLoadAction);
}

void MainWindow::createActions() {
    mpOpenAction = new QAction("Open", this);
    connect(mpOpenAction, SIGNAL(triggered()), this, SLOT(onOpen()));
    mpSaveAction = new QAction("Save", this);
    connect(mpSaveAction, SIGNAL(triggered()), this, SLOT(onSave()));
    mpLoadAction = new QAction("Load", this);
    connect(mpLoadAction, SIGNAL(triggered()), this, SLOT(onLoad()));
}

void MainWindow::onOpen() {
    QString tempFilename = QFileDialog::getOpenFileName(this,
             tr("Open File"), "./", tr("Map Files (*.*)"));
    if( tempFilename.isEmpty() == false ) {
        mpViz->loadMap(tempFilename);
    }
}

void MainWindow::onSave() {
    QString tempFilename = QFileDialog::getSaveFileName(this,
             tr("Save File"), "./", tr("XML Files (*.xml)"));
    if( tempFilename.isEmpty() == false ) {
        mpViz->save(tempFilename);
    }
}

void MainWindow::onLoad() {
    QString tempFilename = QFileDialog::getOpenFileName(this,
             tr("Save File"), "./", tr("XML Files (*.xml)"));
    if( tempFilename.isEmpty() == false ) {
        mpViz->load(tempFilename);
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
   if (event->key() == Qt::Key_R  ) {
       if(mpViz) {
           if(mpViz->mShowSubregion == true) {
               mpViz->mShowSubregion = false;
           }
           else {
               mpViz->mShowSubregion = true;
           }
           repaint();
       }
   }
   else if (event->key() == Qt::Key_S  ) {
       if(mpViz) {
           if(mpViz->mShowSubsegment == true) {
               mpViz->mShowSubsegment = false;
           }
           else {
               mpViz->mShowSubsegment = true;
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
