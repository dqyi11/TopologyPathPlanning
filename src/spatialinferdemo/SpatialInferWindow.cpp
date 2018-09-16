#include <QFileDialog>
#include <QMessageBox>
#include <QtDebug>
#include <QKeyEvent>
#include <QStatusBar>
#include "topologyPathPlanning/spatialinfer/InbetweenRelationFunction.hpp"
#include "topologyPathPlanning/spatialinfer/AvoidRelationFunction.hpp"
#include "topologyPathPlanning/spatialinfer/SideofRelationFunction.hpp"
#include "SpatialInferConfig.hpp"
#include "SpatialInferWindow.hpp"

using namespace std;
using namespace topologyPathPlanning::homotopy;

namespace topologyPathPlanning {

namespace topologyinference {

SpatialInferWindow::SpatialInferWindow( QWidget *parent )
    : QMainWindow( parent ) {
  mpViz = new SpatialInferViz();
  mpMsgBox = new QMessageBox();
  createActions();
  createMenuBar();
  mpStatusLabel = new QLabel();
  statusBar()->addWidget(mpStatusLabel);
  setCentralWidget(mpViz);

  mpConfig = new SpatialInferConfig( this );
  mpConfig->hide();
}

SpatialInferWindow::~SpatialInferWindow() {
  if( mpMsgBox ) {
    delete mpMsgBox;
    mpMsgBox = NULL;
  }
  if( mpConfig ) {
    delete mpConfig;
    mpConfig = NULL;
  }
  if( mpViz ) {
    delete mpViz;
    mpViz = NULL;
  }
}

void SpatialInferWindow::createMenuBar() {
  mpFileMenu = menuBar()->addMenu("&File");
  mpFileMenu->addAction( mpOpenAction );
  mpFileMenu->addAction( mpSaveAction );
  mpFileMenu->addAction( mpLoadAction );

  mpAddMenu = menuBar()->addMenu("&Add");
  mpAddMenu->addAction( mpAddInbetweenSpatialRelationAction );
  mpAddSideofRelationMenu = mpAddMenu->addMenu("&Side-of Relation");
  mpAddSideofRelationMenu->addAction( mpAddLeftofSpatialRelationAction );
  mpAddSideofRelationMenu->addAction( mpAddRightofSpatialRelationAction );
  mpAddSideofRelationMenu->addAction( mpAddTopofSpatialRelationAction );
  mpAddSideofRelationMenu->addAction( mpAddBottomofSpatialRelationAction );
  mpAddMenu->addAction( mpAddAvoidSpatialRelationAction );

  mpManageMenu = menuBar()->addMenu("&Manage");
  mpManageMenu->addAction( mpShowConfigAction ); 
  mpManageMenu->addAction( mpExecuteAction ); 

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
 
  mpAddInbetweenSpatialRelationAction = new QAction("In-between Relation", this);
  connect(mpAddInbetweenSpatialRelationAction, SIGNAL(triggered()), this, SLOT(onAddInbetweenSpatialRelation()));
  mpAddAvoidSpatialRelationAction = new QAction("Avoid Relation", this);
  connect(mpAddAvoidSpatialRelationAction, SIGNAL(triggered()), this, SLOT(onAddAvoidSpatialRelation()));
  mpAddLeftofSpatialRelationAction = new QAction("Left-of Relation", this);
  connect(mpAddLeftofSpatialRelationAction, SIGNAL(triggered()), this, SLOT(onAddLeftofSpatialRelation()));
  mpAddRightofSpatialRelationAction = new QAction("Right-of Relation", this);
  connect(mpAddRightofSpatialRelationAction, SIGNAL(triggered()), this, SLOT(onAddRightofSpatialRelation()));
  mpAddTopofSpatialRelationAction = new QAction("Top-of Relation", this);
  connect(mpAddTopofSpatialRelationAction, SIGNAL(triggered()), this, SLOT(onAddTopofSpatialRelation()));
  mpAddBottomofSpatialRelationAction = new QAction("Bottom-of Relation", this);
  connect(mpAddBottomofSpatialRelationAction, SIGNAL(triggered()), this, SLOT(onAddBottomofSpatialRelation()));

  mpShowConfigAction = new QAction("Show", this);
  connect(mpShowConfigAction, SIGNAL(triggered()), this, SLOT(onShowConfig()));
  mpExecuteAction = new QAction("Execute", this);
  connect(mpExecuteAction, SIGNAL(triggered()), this, SLOT(onExecute()));

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
  else if(event->key() == Qt::Key_PageDown ) {
    if(mpViz) {
      mpViz->nextStringClass();
      repaint();
    }
  }
  else if(event->key() == Qt::Key_PageUp ) {
    if(mpViz) {
      mpViz->prevStringClass();
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
  
void SpatialInferWindow::onAddInbetweenSpatialRelation() {
  vector<Obstacle*> selected_obstacles;
  if( mpViz) {
    selected_obstacles = mpViz->get_selected_obstacles();
  }
  if( selected_obstacles.size() != 2 ) {
    if( mpMsgBox ) {
      mpMsgBox->setText( "Add Inbetween Spatial : Number of obstacles mismatch " );
      mpMsgBox->show();
    } 
    return;
  }
  InBetweenRelationFunction* p_func = new InBetweenRelationFunction();
  for( unsigned int i=0; i < selected_obstacles.size(); i++ ) {
    Obstacle* p_obs = selected_obstacles[i];
    p_func->mp_obstacles.push_back( p_obs );
  }
  if( mpViz ) {
    mpViz->get_spatial_relation_mgr()->mp_functions.push_back( p_func );
  } 
  mpViz->clear_selected_obstacles();
  repaint(); 
}

void SpatialInferWindow::onAddAvoidSpatialRelation() {
  vector<Obstacle*> selected_obstacles;
  if( mpViz) {
    selected_obstacles = mpViz->get_selected_obstacles();
  }
  if( selected_obstacles.size() != 1 ) {
    if( mpMsgBox ) {
      mpMsgBox->setText( "Add Avoid Spatial : Number of obstacles mismatch " );
      mpMsgBox->show();
    } 
    return;
  }
  AvoidRelationFunction* p_func = new AvoidRelationFunction();
  p_func->mp_obstacle = selected_obstacles[0];
  if( mpViz ) {
    mpViz->get_spatial_relation_mgr()->mp_functions.push_back( p_func );
  } 
  mpViz->clear_selected_obstacles();
  repaint();
  
  if( mpConfig ) {
    mpConfig->updateDisplay();
  }
}

void SpatialInferWindow::onAddLeftofSpatialRelation() {
  vector<Obstacle*> selected_obstacles;
  if( mpViz) {
    selected_obstacles = mpViz->get_selected_obstacles();
  }
  if( selected_obstacles.size() != 1 ) {
    if( mpMsgBox ) {
      mpMsgBox->setText( "Add Left-of Spatial : Number of obstacles mismatch " );
      mpMsgBox->show();
    } 
    return;
  }
  SideOfRelationFunction* p_func = new SideOfRelationFunction( SIDE_TYPE_LEFT );
  p_func->mp_obstacle = selected_obstacles[0];
  if( mpViz ) {
    mpViz->get_spatial_relation_mgr()->mp_functions.push_back( p_func );
  } 
  mpViz->clear_selected_obstacles();
  repaint(); 
}

void SpatialInferWindow::onAddRightofSpatialRelation() {
  vector<Obstacle*> selected_obstacles;
  if( mpViz) {
    selected_obstacles = mpViz->get_selected_obstacles();
  }
  if( selected_obstacles.size() != 1 ) {
    if( mpMsgBox ) {
      mpMsgBox->setText( "Add Right-of Spatial : Number of obstacles mismatch " );
      mpMsgBox->show();
    } 
    return;
  }
  SideOfRelationFunction* p_func = new SideOfRelationFunction( SIDE_TYPE_RIGHT );
  p_func->mp_obstacle = selected_obstacles[0];
  if( mpViz ) {
    mpViz->get_spatial_relation_mgr()->mp_functions.push_back( p_func );
  } 
  mpViz->clear_selected_obstacles();
  repaint(); 
}

void SpatialInferWindow::onAddTopofSpatialRelation() {
  vector<Obstacle*> selected_obstacles;
  if( mpViz) {
    selected_obstacles = mpViz->get_selected_obstacles();
  }
  if( selected_obstacles.size() != 1 ) {
    if( mpMsgBox ) {
      mpMsgBox->setText( "Add Top-of Spatial : Number of obstacles mismatch " );
      mpMsgBox->show();
    } 
    return;
  }
  SideOfRelationFunction* p_func = new SideOfRelationFunction( SIDE_TYPE_TOP );
  p_func->mp_obstacle = selected_obstacles[0];
  if( mpViz ) {
    mpViz->get_spatial_relation_mgr()->mp_functions.push_back( p_func );
  } 
  mpViz->clear_selected_obstacles();
  repaint(); 
}

void SpatialInferWindow::onAddBottomofSpatialRelation() {
  vector<Obstacle*> selected_obstacles;
  if( mpViz) {
    selected_obstacles = mpViz->get_selected_obstacles();
  }
  if( selected_obstacles.size() != 1 ) {
    if( mpMsgBox ) {
      mpMsgBox->setText( "Add Bottom-of Spatial : Number of obstacles mismatch " );
      mpMsgBox->show();
    } 
    return;
  }
  SideOfRelationFunction* p_func = new SideOfRelationFunction( SIDE_TYPE_BOTTOM );
  p_func->mp_obstacle = selected_obstacles[0];
  if( mpViz ) {
    mpViz->get_spatial_relation_mgr()->mp_functions.push_back( p_func );
  }
  mpViz->clear_selected_obstacles();
  repaint(); 
}

void SpatialInferWindow::onShowConfig() {
  if( mpConfig ) {
    mpConfig->updateDisplay();
    mpConfig->exec();
  }
}

void SpatialInferWindow::onExecute() {
  if( mpViz->get_spatial_relation_mgr()->m_start_x < 0 ||
      mpViz->get_spatial_relation_mgr()->m_start_y < 0 ) {
    if( mpMsgBox ) {
      mpMsgBox->setText( "Start position not set" );
      mpMsgBox->show();
    }
  } 

  if( mpViz->get_spatial_relation_mgr()->m_goal_x < 0 ||
      mpViz->get_spatial_relation_mgr()->m_goal_y < 0 ) {
    if( mpMsgBox ) {
      mpMsgBox->setText( "Goal position not set" );
      mpMsgBox->show();
    } 
  }

  mpViz->get_spatial_relation_mgr()->get_string_classes( mpViz->get_reference_frame_set() ); 
  repaint();
}


} // topologyinference

} // topologyPathPlanning
