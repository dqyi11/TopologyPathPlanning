#include "spatialinfer_window.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QtDebug>
#include <QKeyEvent>
#include <QStatusBar>

using namespace homotopy;
using namespace topology_inference;

SpatialInferWindow::SpatialInferWindow(QWidget *parent)
    : QMainWindow(parent) {
  mpViz = new SpatialInferViz();
  mpMsgBox = new QMessageBox();
  createActions();
  createMenuBar();
  mpStatusLabel = new QLabel();
  statusBar()->addWidget(mpStatusLabel);
  setCentralWidget(mpViz);
}

SpatialInferWindow::~SpatialInferWindow() {
  if(mpMsgBox) {
    delete mpMsgBox;
    mpMsgBox = NULL;
  }
  if(mpViz) {
    delete mpViz;
    mpViz = NULL;
  }
}

void SpatialInferWindow::createMenuBar() {
  mpFileMenu = menuBar()->addMenu("&File");
  mpFileMenu->addAction( mpOpenAction );
  mpFileMenu->addAction( mpSaveAction );
  mpFileMenu->addAction( mpLoadAction );

  mpContextMenu = new QMenu();
  setContextMenuPolicy( Qt::CustomContextMenu );
  mpContextMenu->addAction( mpAddStartAction );
  mpContextMenu->addAction( mpAddGoalAction );
}

void SpatialInferWindow::createActions() {
  mpOpenAction = new QAction("Open", this);
  connect(mpOpenAction, SIGNAL(triggered()), this, SLOT(onOpen()));
  mpSaveAction = new QAction("Save", this);
  connect(mpSaveAction, SIGNAL(triggered()), this, SLOT(onSave()));
  mpLoadAction = new QAction("Load", this);
  connect(mpLoadAction, SIGNAL(triggered()), this, SLOT(onLoad()));

  mpAddStartAction = new QAction("Add Start", this);
  mpAddGoalAction = new QAction("Add Goal", this);
  connect( mpAddStartAction, SIGNAL(triggered()), this, SLOT(onAddStart()) );
  connect( mpAddGoalAction, SIGNAL(triggered()), this, SLOT(onAddGoal()) );
 
  connect( this, SIGNAL(customContextMenuRequested(const QPoint)), this, SLOT(contextMenuRequested(QPoint)) ); 
}

void SpatialInferWindow::contextMenuRequested( QPoint point ) {
  mCursorPoint = point;
  mpContextMenu->popup( mapToGlobal( point ) );
}

void SpatialInferWindow::onOpen() {
  QString tempFilename = QFileDialog::getOpenFileName(this,
          tr("Open File"), "./", tr("Map Files (*.*)"));
  if( tempFilename.isEmpty() == false ) {
    mpViz->loadMap(tempFilename);
    updateStatusBar();
  }
}

void SpatialInferWindow::onSave() {
  QString tempFilename = QFileDialog::getSaveFileName(this,
          tr("Save File"), "./", tr("XML Files (*.xml)"));
  if( tempFilename.isEmpty() == false ) {
      mpViz->save(tempFilename);
  }
}

void SpatialInferWindow::onLoad() {
  QString tempFilename = QFileDialog::getOpenFileName(this,
          tr("Save File"), "./", tr("XML Files (*.xml)"));
  if( tempFilename.isEmpty() == false ) {
      mpViz->load(tempFilename);
  }
}

void SpatialInferWindow::keyPressEvent(QKeyEvent *event) {
  if (event->key() == Qt::Key_Space  ) {
    if(mpViz) {
      if(mpViz->getMode() == SUBREGION ) {
        mpViz->setMode( LINE_SUBSEGMENT );
      }
      else {
        mpViz->setMode( SUBREGION );
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
  else if(event->key() == Qt::Key_Up ) {
    if(mpViz) {
      if( mpViz->getMode() == SUBREGION ) {
        mpViz->nextRegion();
      }
      else {
        mpViz->nextLineSubsegmentSet();
      }
      updateStatusBar();
      repaint();
    }
  }
  else if(event->key() == Qt::Key_Down ) {
    if(mpViz) {
      if( mpViz->getMode() == SUBREGION ) {
        mpViz->prevRegion();
      }
      else {
        mpViz->prevLineSubsegmentSet();
      }
      updateStatusBar();
      repaint();
    }
  }
  else if(event->key() == Qt::Key_Right ) {
    if(mpViz) {
      if( mpViz->getMode() == SUBREGION ) {
        mpViz->nextSubregion();
      }
      else {
        mpViz->nextLineSubsegment();
      }
      updateStatusBar();
      repaint();
    }
  }
  else if(event->key() == Qt::Key_Left ) {
    if(mpViz) {
      if( mpViz->getMode() == SUBREGION ) {
        mpViz->prevSubregion();
      }
      else {
        mpViz->prevLineSubsegment();
      }
      updateStatusBar();
      repaint();
    }
  }
}

void SpatialInferWindow::updateStatusBar() {

  if(mpStatusLabel) {
    QString status = "";
    if( mpViz->getSelectedRegion() ) {
      status += "Region (" + QString::number(mpViz->getRegionIdx()) + ")";
      if ( mpViz->getSelectedSubregion() ) {
        status += "- (" + QString::number(mpViz->getSubregionIdx()) + ")";
        status += " = ";
        for( unsigned int i = 0; i < mpViz->getSelectedSubregion()->m_neighbors.size(); i ++ ) {
          LineSubSegment* p_line_subseg = mpViz->getSelectedSubregion()->m_neighbors[i];
          status += " [ " + QString::fromStdString(p_line_subseg->get_name())  + " ] ";
        } 
      }
      else {
        status += " = ";
        if ( mpViz->getSelectedRegion() ) {
          status += " [ " + QString::fromStdString(mpViz->getSelectedRegion()->mp_line_segments_a->get_name())  + " ] ";
          status += " [ " + QString::fromStdString(mpViz->getSelectedRegion()->mp_line_segments_b->get_name())  + " ] ";
        }
      }
    }
    mpStatusLabel->setText(status);
  }
  repaint();
}

void SpatialInferWindow::onAddStart() {
  if( mpViz ) {
    if( mpViz->get_spatial_relation_mgr() ) {
      mpViz->get_spatial_relation_mgr()->m_start_x = mCursorPoint.x();
      mpViz->get_spatial_relation_mgr()->m_start_y = mCursorPoint.y();
      repaint();
    }
  }
}

void SpatialInferWindow::onAddGoal() {
  if( mpViz ) {
    if( mpViz->get_spatial_relation_mgr() ) {
      mpViz->get_spatial_relation_mgr()->m_goal_x = mCursorPoint.x();
      mpViz->get_spatial_relation_mgr()->m_goal_y = mCursorPoint.y();
      repaint();
    }
  }

}
