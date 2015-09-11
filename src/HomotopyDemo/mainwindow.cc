#include "mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QtDebug>
#include <QKeyEvent>
#include <QStatusBar>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent) {
    mpViz = new HomotopyViz();
    mpMsgBox = new QMessageBox();
    createActions();
    createMenuBar();
    mpStatusLabel = new QLabel();
    statusBar()->addWidget(mpStatusLabel);
    setCentralWidget(mpViz);
}

MainWindow::~MainWindow() {
    if(mpMsgBox) {
        delete mpMsgBox;
        mpMsgBox = NULL;
    }
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
        updateStatusBar();
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
           updateStatusBar();
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
           updateStatusBar();
           repaint();
       }
   }
   else if (event->key() == Qt::Key_X  ) {
       if(mpViz) {
           QString ref_str = mpViz->generate_string();
           if (mpMsgBox) {
               mpMsgBox->setText(ref_str);
               mpMsgBox->show();
           }
       }
   } 
   else if(event->key() == Qt::Key_Up ) {
       if(mpViz) {
           mpViz->nextRegion();
           updateStatusBar();
           repaint();
       }
   }
   else if(event->key() == Qt::Key_Down ) {
       if(mpViz) {
           mpViz->prevRegion();
           updateStatusBar();
           repaint();
       }
   }
   else if(event->key() == Qt::Key_Right ) {
       if(mpViz) {
           mpViz->nextSubregion();
           updateStatusBar();
           repaint();
       }
   }
   else if(event->key() == Qt::Key_Left ) {
       if(mpViz) {
           mpViz->prevSubregion();
           updateStatusBar();
           repaint();
       }
   }
}

void MainWindow::updateStatusBar() {

    if(mpStatusLabel) {
        QString status = "";
        status += "Region (" + QString::number(mpViz->getRegionIdx()) + ")";
        if ( mpViz->mShowSubregion ) {
            status += "- (" + QString::number(mpViz->getSubregionIdx()) + ")";
            status += " = ";
            if ( mpViz->getSelectedSubregion() ) {
                for( unsigned int i = 0; i < mpViz->getSelectedSubregion()->m_neighbors.size(); i ++ ) {
                    LineSubSegment* p_line_subseg = mpViz->getSelectedSubregion()->m_neighbors[i];
                    status += " [ " + QString::fromStdString(p_line_subseg->get_name())  + " ] ";
                } 
            }
        }
        else {
            status += " = ";
            if ( mpViz->getSelectedRegion() ) {
                status += " [ " + QString::fromStdString(mpViz->getSelectedRegion()->mp_line_segments_a->get_name())  + " ] ";
                status += " [ " + QString::fromStdString(mpViz->getSelectedRegion()->mp_line_segments_b->get_name())  + " ] ";
            }

        }
        
        mpStatusLabel->setText(status);
    }
    repaint();
}
