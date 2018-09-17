#include <QFile>
#include <QTextStream>
#include <QFileDialog>
#include <QMessageBox>
#include <QtDebug>
#include <QKeyEvent>
#include <QStatusBar>
#include <QApplication>

#include "topologyPathPlanning/homotopy/ImgLoadUtil.hpp"
#include "topologyPathPlanning/tarrts/ExpandingTree.hpp"
#include "MLRRTstarConfig.hpp"
#include "MLRRTstarWindow.hpp"

using namespace std;
using namespace topologyPathPlanning::homotopy;

namespace topologyPathPlanning {

namespace tarrts {

MLRRTstarWindow::MLRRTstarWindow(QWidget* parent)
    : QMainWindow(parent) {

  mShowObj = false;

  mpViz = new MLRRTstarViz();
  createActions();
  createMenuBar();

  mpMap = NULL;
  mpMLRRTstar = NULL;
  mpReferenceFrameSet = NULL;

  mpMLRRTstarConfig = new MLRRTstarConfig(this);
  mpMLRRTstarConfig->hide();
  setCentralWidget(mpViz);

  mpStatusLabel = new QLabel();
  mpStatusLabel->setFixedWidth(180);
  mpStatusProgressBar = new QProgressBar();
  mpStringClassLabel = new QLabel();
  mpStringClassLabel->setFixedWidth(120);

  statusBar()->addWidget(mpStatusLabel);
  statusBar()->addWidget(mpStatusProgressBar);
  statusBar()->addWidget(mpStringClassLabel);
  updateTitle();
}

MLRRTstarWindow::~MLRRTstarWindow() {
  if(mpMLRRTstarConfig) {
    delete mpMLRRTstarConfig;
    mpMLRRTstarConfig = NULL;
  }
  if(mpMLRRTstar) {
    delete mpMLRRTstar;
    mpMLRRTstar = NULL;
  }
  if(mpReferenceFrameSet) {
    delete mpReferenceFrameSet;
    mpReferenceFrameSet = NULL;
  }
  if(mpViz) {
    delete mpViz;
    mpViz = NULL;
  }
}

void MLRRTstarWindow::createMenuBar() {
  mpFileMenu = menuBar()->addMenu("&File");
  mpFileMenu->addAction(mpOpenAction);
  mpFileMenu->addAction(mpSaveAction);
  mpFileMenu->addAction(mpExportAction);
  mpFileMenu->addAction(mpExportPathAction);

  mpEditMenu = menuBar()->addMenu("&Edit");
  mpEditMenu->addAction(mpLoadMapAction);
  mpEditMenu->addAction(mpLoadObjAction);
  mpEditMenu->addAction(mpRunAction);
  mpEditMenu->addAction(mpResetAction);

  mpToolMenu = menuBar()->addMenu("&Tool");
  mpToolMenu->addAction(mpSaveScreenAction);
  mpToolMenu->addAction(mpExportGrammarGraphAction);
  mpToolMenu->addAction(mpExportAllSimpleStringsAction);
  mpToolMenu->addAction(mpExportStringClassHistAction);

  mpContextMenu = new QMenu();
  setContextMenuPolicy(Qt::CustomContextMenu);

  mpContextMenu->addAction(mpAddStartAction);
  mpContextMenu->addAction(mpAddGoalAction);
}

void MLRRTstarWindow::createActions() {

  mpOpenAction = new QAction("Open", this);
  mpSaveAction = new QAction("Save", this);
  mpExportAction = new QAction("Export", this);
  mpExportPathAction = new QAction("Export Path", this);
  mpExportStringClassHistAction = new QAction("Export String Class Hist", this);
  mpLoadMapAction = new QAction("Load Map", this);
  mpLoadObjAction = new QAction("Config Objective", this);
  mpRunAction = new QAction("Run", this);
  mpResetAction = new QAction("Reset", this);

  connect(mpOpenAction, SIGNAL(triggered()), this, SLOT(onOpen()));
  connect(mpSaveAction, SIGNAL(triggered()), this, SLOT(onSave()));
  connect(mpExportAction, SIGNAL(triggered()), this, SLOT(onExport()));
  connect(mpExportPathAction, SIGNAL(triggered()), this, SLOT(onExportPath()));
  connect(mpExportStringClassHistAction, SIGNAL(triggered()), this, SLOT(onExportStringClassHist()));

  connect(mpLoadMapAction, SIGNAL(triggered()), this, SLOT(onLoadMap()));
  connect(mpLoadObjAction, SIGNAL(triggered()), this, SLOT(onLoadObj()));
  connect(mpRunAction, SIGNAL(triggered()), this, SLOT(onRun()));
  connect(mpResetAction, SIGNAL(triggered()), this, SLOT(onReset()));

  mpAddStartAction = new QAction("Add Start", this);
  mpAddGoalAction = new QAction("Add Goal", this);
  connect(mpAddStartAction, SIGNAL(triggered()), this, SLOT(onAddStart()));
  connect(mpAddGoalAction, SIGNAL(triggered()), this, SLOT(onAddGoal()));

  mpSaveScreenAction = new QAction("Save Screen", this);
  mpExportGrammarGraphAction = new QAction("Export Grammar", this);
  mpExportAllSimpleStringsAction = new QAction("Export All Simple Strings", this);
  connect(mpSaveScreenAction, SIGNAL(triggered()), this, SLOT(onSaveScreen()));
  connect(mpExportGrammarGraphAction, SIGNAL(triggered()), this, SLOT(onExportGrammar()));
  connect(mpExportAllSimpleStringsAction, SIGNAL(triggered()), this, SLOT(onExportAllSimpleStrings()));

  connect(this, SIGNAL(customContextMenuRequested(const QPoint)),this, SLOT(contextMenuRequested(QPoint)));
}

void MLRRTstarWindow::onOpen() {
  QString tempFilename = QFileDialog::getOpenFileName(this,
             tr("Open File"), "./", tr("XML Files (*.xml)"));

  if(setupPlanning(tempFilename)) {
    repaint();
  }
}

bool MLRRTstarWindow::exportPaths() {
  if(mpViz) {
    bool success = false;
    success = mpViz->m_PPInfo.exportPaths(mpViz->m_PPInfo.mPathsOutput);
    success = mpViz->drawPath(mpViz->m_PPInfo.mPathsOutput+".png");
    return success;
  }
  return false;
}

bool MLRRTstarWindow::setupPlanning(QString filename) {
  if (mpViz) {
    mpViz->m_PPInfo.loadFromFile(filename);
    openMap(mpViz->m_PPInfo.mMapFullpath);
    if(mpMLRRTstarConfig) {
      mpMLRRTstarConfig->updateDisplay();
    }
    return true;
  }
  return false;
}

void MLRRTstarWindow::setShowObj( bool show ) {
  mShowObj = show;
  if( mShowObj ) {
    if( mpViz ) {
      if( mpViz->m_PPInfo.mMinDistEnabled == false ) {
        if(mpViz->m_PPInfo.mp_obj==NULL) {
          mpViz->m_PPInfo.initObjPixmap();
        }
        mpViz->setPixmap( *(mpViz->m_PPInfo.mp_obj) );
        repaint();
      }
    }
  }
  else {
    if( mpViz) {
      mpViz->setPixmap( *mpMap );
      repaint();
    }
  }
}

void MLRRTstarWindow::onSave() {
  QString tempFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("XML Files (*.xml)"));

  if(mpViz) {
    mpViz->m_PPInfo.saveToFile(tempFilename);
  }
}

void MLRRTstarWindow::onExport() {
  QString pathFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("Txt Files (*.txt)"));
  if (pathFilename != "") {
    mpViz->m_PPInfo.mPathsOutput = pathFilename;
    std::vector<Path*> p_paths = mpMLRRTstar->getPaths();
    mpViz->m_PPInfo.loadPaths(p_paths);
    exportPaths();
  }
}

void MLRRTstarWindow::onExportPath() {

  QString pathFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("Txt Files (*.txt)"));
  if (pathFilename != "") {

    Path* p_viz_path = mpViz->getVizPath();
    if( p_viz_path ) {
      exportPath( p_viz_path, pathFilename );
    }
  }
}

void MLRRTstarWindow::onLoadMap() {
  QString tempFilename = QFileDialog::getOpenFileName(this,
             tr("Open Map File"), "./", tr("Map Files (*.*)"));

  QFileInfo fileInfo(tempFilename);
  QString filename(fileInfo.fileName());
  mpViz->m_PPInfo.mMapFilename = filename;
  mpViz->m_PPInfo.mMapFullpath = tempFilename;
  qDebug("OPENING ");
  //qDebug(mpViz->m_PPInfo.m_map_filename.toStdString().c_str());

  openMap(mpViz->m_PPInfo.mMapFullpath);
}


bool MLRRTstarWindow::openMap(QString filename) {
  if(mpMap) {
    delete mpMap;
    mpMap = NULL;
  }
  mpMap = new QPixmap(filename);
  if(mpMap) {
    mpViz->m_PPInfo.mMapWidth = mpMap->width();
    mpViz->m_PPInfo.mMapHeight = mpMap->height();
    mpViz->setPixmap(*mpMap);
    updateTitle();

    int map_width = 0, map_height = 0;
    std::vector< std::vector< Point2D > > obstacles;
    mpReferenceFrameSet = new ReferenceFrameSet();
    loadMapInfo( filename.toStdString(), map_width, map_height, obstacles );
    mpReferenceFrameSet->init( map_width, map_height, obstacles );
    mpViz->setReferenceFrameSet( mpReferenceFrameSet );
    return true;
  }
  return false;
}

void MLRRTstarWindow::onLoadObj() {
  mpMLRRTstarConfig->exec();
  updateTitle();
}

void MLRRTstarWindow::onRun() {
  if (mpViz->m_PPInfo.mMapWidth <= 0 || mpViz->m_PPInfo.mMapHeight <= 0) {
    QMessageBox msgBox;
    msgBox.setText("Map is not initialized.");
    msgBox.exec();
    return;
  }
  if(mpViz->m_PPInfo.mStart.x()<0 || mpViz->m_PPInfo.mStart.y()<0) {
    QMessageBox msgBox;
    msgBox.setText("Start is not set.");
    msgBox.exec();
    return;
  }
  if(mpViz->m_PPInfo.mGoal.x()<0 || mpViz->m_PPInfo.mGoal.y()<0) {
    QMessageBox msgBox;
    msgBox.setText("Goal is not set.");
    msgBox.exec();
    return;
  }

  planPath();
  repaint();
}

void MLRRTstarWindow::onExportGrammar() {

  QString grammarFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("DOT Files (*.dot)"));
  if( mpReferenceFrameSet ) {
    StringGrammar* p_grammar = mpReferenceFrameSet->getStringGrammar( mpViz->m_PPInfo.mStart.x(), mpViz->m_PPInfo.mStart.y(), mpViz->m_PPInfo.mGoal.x(), mpViz->m_PPInfo.mGoal.y() );
    if( p_grammar ) {
      p_grammar->output( grammarFilename.toStdString() );
      ExpandingTree tree;
      tree.init( p_grammar );
      QString expandingTreeFilename = grammarFilename.replace(".dot","") + "-exp.dot";
      tree.output( expandingTreeFilename.toStdString() );
    }
  }
}

void MLRRTstarWindow::planPath() {

  if(mpMLRRTstar==NULL) {
      mpViz->m_PPInfo.initFuncParam();
      QString msg = "INIT RRTstar ... \n";
      msg += "SegmentLen( " + QString::number(mpViz->m_PPInfo.mSegmentLength) + " ) \n";
      msg += "MaxIterationNum( " + QString::number(mpViz->m_PPInfo.mMaxIterationNum) + " ) \n";
      qDebug() << msg;

      mpMLRRTstar = new MLRRTstar(mpMap->width(), mpMap->height(), mpViz->m_PPInfo.mSegmentLength);
      mpMLRRTstar->setReferenceFrames( mpReferenceFrameSet );
      if( mpViz->m_PPInfo.mHomotopicEnforcement ) {
        mpMLRRTstar->setHomotopicEnforcement( true );
      }
      else {
        mpMLRRTstar->setHomotopicEnforcement( false );
      }
      POS2D start(mpViz->m_PPInfo.mStart.x(), mpViz->m_PPInfo.mStart.y());
      POS2D goal(mpViz->m_PPInfo.mGoal.x(), mpViz->m_PPInfo.mGoal.y());

      mpMLRRTstar->init(start, goal, mpViz->m_PPInfo.mpFunc, mpViz->m_PPInfo.mCostDistribution, mpViz->m_PPInfo.mGrammarType);
      mpViz->m_PPInfo.getObstacleInfo(mpMLRRTstar->getMapInfo());
      mpViz->setTree(mpMLRRTstar);

      mpMLRRTstar->getExpandingTreeMgr()->getExpandingTree()->output( "output.dot" );
      mpMLRRTstar->getExpandingTreeMgr()->getExpandingTree()->print();
      mpMLRRTstar->getExpandingTreeMgr()->exportSubregionMgrs("subregion_mgrs.txt");
  }

  mpViz->setFinishedPlanning( false );

  while( ( false == mpViz->isFinishedPlanning() )
         && mpMLRRTstar->getCurrentIteration() <= mpViz->m_PPInfo.mMaxIterationNum) {
    QString msg = "CurrentIteration " + QString::number(mpMLRRTstar->getCurrentIteration()) + " ";
    mpMLRRTstar->extend();
    msg += QString::number(mpMLRRTstar->getExpandingTreeMgr()->getStringClasses().size());
    qDebug() << msg;

    mpMLRRTstar->updatePaths();

    QApplication::processEvents();

    updateStatus();
    repaint();
  }

  qDebug() << "START MERGE ";
  //mpMLRRTstar->get_string_class_mgr()->merge();
  qDebug() << "END MERGE ";
  //Path* path = mpMLRRTstar->find_path();

  mpViz->setFinishedPlanning( true );
  repaint();
}

void MLRRTstarWindow::onAddStart() {
  mpViz->m_PPInfo.mStart = mCursorPoint;
  repaint();
}

void MLRRTstarWindow::onAddGoal() {
  mpViz->m_PPInfo.mGoal = mCursorPoint;
  repaint();
}

void MLRRTstarWindow::onSaveScreen() {
  QString tempFilename = QFileDialog::getSaveFileName(this, tr("Save PNG File"), "./", tr("PNG Files (*.png)"));
  mpViz->saveCurrentViz( tempFilename );
}

void MLRRTstarWindow::contextMenuRequested(QPoint point) {
  mCursorPoint = point;
  mpContextMenu->popup(mapToGlobal(point));
}

void MLRRTstarWindow::updateTitle() {
  QString title = mpViz->m_PPInfo.mMapFilename;
  setWindowTitle(title);
}

void MLRRTstarWindow::updateStatus() {
  if(mpViz==NULL) {
    return;
  }
  if(mpStatusProgressBar) {
    if(mpMLRRTstar) {
      mpStatusProgressBar->setMinimum(0);
      mpStatusProgressBar->setMaximum(mpViz->m_PPInfo.mMaxIterationNum);
      mpStatusProgressBar->setValue(mpMLRRTstar->getCurrentIteration());
    }
  }
  if(mpStatusLabel) {
    QString status = "";
    if (mpViz->isFinishedPlanning() == false) {
      status += QString::fromStdString(mpViz->getSubregionName());
      status += " || ";
      status += QString::fromStdString(mpViz->getReferenceFrameName());
    }
    else {
      status += QString::number( mpViz->getFoundPathIndex() );
      status += " / ";
      status += QString::number( mpViz->m_PPInfo.mpFoundPaths.size() );
    }
    mpStatusLabel->setText(status);
  }
  if(mpStringClassLabel) {
    QString status = mpViz->getStringClassInfo();
    mpStringClassLabel->setText(status);
  }
  repaint();
}

void MLRRTstarWindow::keyPressEvent(QKeyEvent *event) {
   if ( event->key() == Qt::Key_Escape ) {
     if( mpViz ) {
       mpViz->setFinishedPlanning( true );
     }
     updateStatus();
     repaint();
   }
   else if ( event->key() == Qt::Key_R  ) {
     if(mpViz) {
       if(mpViz->showReferenceFrames() == true) {
         mpViz->setShowReferenceFrames( false );
       }
       else {
         mpViz->setShowReferenceFrames( true );
       }
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_S ) {
     if(mpViz) {
       if(mpViz->showSubregions() == true) {
         mpViz->setShowSubregions( false );
       }
       else {
         mpViz->setShowSubregions( true );
       }
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_T ) {
     if(mpViz) {
       if(mpViz->showTree() == true) {
         mpViz->setShowTree( false );
       }
       else {
         mpViz->setShowTree( true );
       }
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_P ) {
     if(mpViz) {
       if(mpViz->showPaths() == true) {
         mpViz->setShowPaths( false );
       }
       else {
         mpViz->setShowPaths( true );
       }
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_I ) {
     if(mpViz) {
       if( mpViz->getDrawedPoints().size() > 1 ) {
         mpViz->importStringConstraint( mpViz->getDrawedPoints(), mpViz->m_PPInfo.mGrammarType );
         mpViz->setShowDrawedPoints(false);
         mpViz->setMode( NORMAL );
       }
     }
     updateStatus();
     repaint();
   }
   else if ( event->key() == Qt::Key_O ) {
     if( getShowObj() ) {
       setShowObj( false );
     }
     else {
       setShowObj( true );
     }
   }
   else if ( event->key() == Qt::Key_Space ) {
     if( mpViz ) {
       if( mpViz->getMode() == NORMAL ) {
         mpViz->setMode( DRAWING );
       }
       else{
         mpViz->setMode( NORMAL );
       }
     }
   }
   else if ( event->key() == Qt::Key_Up ) {
     if(mpViz) {
       mpViz->prevReferenceFrame();
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_Down ) {
     if(mpViz) {
       mpViz->nextReferenceFrame();
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_Left ) {
     if(mpViz) {
       mpViz->prevFoundPath();
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_Right ) {
     if(mpViz) {
       mpViz->nextFoundPath();
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_PageUp ) {
     if(mpViz) {
       mpViz->prevStringClass();
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_PageDown ) {
     if(mpViz) {
       mpViz->nextStringClass();
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_Q ) {
     if(mpViz) {
       mpViz->prevExpNode();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_W ) {
     if(mpViz) {
       mpViz->nextExpNode();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_Z ) {
     if(mpViz) {
       mpViz->prevSubregion();
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_X ) {
     if(mpViz) {
       mpViz->nextSubregion();
       updateStatus();
       repaint();
     }
   }

}

void MLRRTstarWindow::onExportAllSimpleStrings() {

  QString stringFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("TXT Files (*.txt)"));
  if( mpReferenceFrameSet ) {
    std::vector< std::vector< std::string > > simple_strings;
    StringGrammar* p_grammar = mpReferenceFrameSet->getStringGrammar( mpViz->m_PPInfo.mStart.x(), mpViz->m_PPInfo.mStart.y(), mpViz->m_PPInfo.mGoal.x(), mpViz->m_PPInfo.mGoal.y() );
    if( p_grammar ) {
      simple_strings = p_grammar->getAllSimpleStrings();
      QFile file(stringFilename);
      if( !file.open(QIODevice::WriteOnly | QIODevice::Text) ) {
        return;
      }
      QTextStream out(&file);
      for( std::vector< std::vector< std::string > >::iterator it = simple_strings.begin(); it != simple_strings.end(); it++ ) {
        std::vector< std::string > ids = (*it);
        for( unsigned int i=0; i < ids.size(); i ++) {
          out << ids[i].c_str() << " ";
        }
        if( p_grammar->isValidString( ids )) {
          out << "VALID";
        }
        else{
          out << "INVALID";
        }
        out << "\n";
      }
    }
  }
}

bool MLRRTstarWindow::exportPath( Path* path, QString filename ) {
  if( path ) {
    QFile file( filename );
    if( file.open( QIODevice::ReadWrite ) ) {
      QTextStream stream( &file );
      //stream << path->m_start[0] << " " << path->m_start[1] << "\n";
      for( unsigned int i=0; i < path->mWaypoints.size(); i++ ) {
        stream << path->mWaypoints[i][0] << " " << path->mWaypoints[i][1] << "\n";
      }
      //stream << path->m_goal[0] << " " << path->m_goal[1] << "\n";
    }
  }
  return false;
}

void MLRRTstarWindow::onReset() {

    if(mpMLRRTstar) {
      delete mpMLRRTstar;
      mpMLRRTstar = NULL;
    }
    if (mpViz) {
      for( std::vector<Path*>::iterator it = mpViz->m_PPInfo.mpFoundPaths.begin();
           it != mpViz->m_PPInfo.mpFoundPaths.end(); it ++ ) {
        Path * p_path = (*it);
        delete p_path;
        p_path = NULL;
      }
      mpViz->m_PPInfo.mpFoundPaths.clear();
    }

}

void MLRRTstarWindow::onExportStringClassHist() {
  QString pathFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("Txt Files (*.txt)"));
  if(pathFilename != "") {
    if(mpMLRRTstar) {
      mpMLRRTstar->getExpandingTreeMgr()->dumpHistoricalData( pathFilename.toStdString() );
    }
  }
}

} // tarrts

} // topologyPathPlanning
