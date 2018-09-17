#include <QFile>
#include <QTextStream>
#include <QFileDialog>
#include <QMessageBox>
#include <QtDebug>
#include <QKeyEvent>
#include <QStatusBar>
#include <QApplication>

#include "topologyPathPlanning/homotopy/ImgLoadUtil.hpp"
#include "BiRRTstarConfig.hpp"
#include "BiRRTstarWindow.hpp"

using namespace topologyPathPlanning::homotopy;

namespace topologyPathPlanning {

namespace harrts{

BIRRTstarWindow::BIRRTstarWindow(QWidget *parent)
    : QMainWindow(parent) {
  mpViz = new BIRRTstarViz();

  createActions();
  createMenuBar();

  mpMap = NULL;
  mpBIRRTstar = NULL;
  mpReferenceFrameSet = NULL;

  mpBIRRTstarConfig = new BIRRTstarConfig(this);
  mpBIRRTstarConfig->hide();

  setCentralWidget(mpViz);

  mpStatusLabel = new QLabel();
  mpStatusLabel->setFixedWidth(120);
  mpStatusProgressBar = new QProgressBar();
    
  statusBar()->addWidget(mpStatusLabel);
  statusBar()->addWidget(mpStatusProgressBar);
  updateTitle();
}

BIRRTstarWindow::~BIRRTstarWindow() {
  if(mpBIRRTstarConfig) {
    delete mpBIRRTstarConfig;
    mpBIRRTstarConfig = NULL;
  }
  if(mpBIRRTstar) {
    delete mpBIRRTstar;
    mpBIRRTstar = NULL;
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

void BIRRTstarWindow::createMenuBar() {
  mpFileMenu = menuBar()->addMenu("&File");
  mpFileMenu->addAction(mpOpenAction);
  mpFileMenu->addAction(mpSaveAction);
  mpFileMenu->addAction(mpExportAction);

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

void BIRRTstarWindow::createActions() {
  mpOpenAction = new QAction("Open", this);
  mpSaveAction = new QAction("Save", this);
  mpExportAction = new QAction("Export", this);
  mpExportStringClassHistAction = new QAction("Export String Class Hist", this);
  mpLoadMapAction = new QAction("Load Map", this);
  mpLoadObjAction = new QAction("Config Objective", this);
  mpRunAction = new QAction("Run", this);
  mpResetAction = new QAction("Reset", this);

  connect(mpOpenAction, SIGNAL(triggered()), this, SLOT(onOpen()));
  connect(mpSaveAction, SIGNAL(triggered()), this, SLOT(onSave()));
  connect(mpExportAction, SIGNAL(triggered()), this, SLOT(onExport()));
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

void BIRRTstarWindow::onOpen() {
  QString tempFilename = QFileDialog::getOpenFileName(this,
             tr("Open File"), "./", tr("XML Files (*.xml)"));

  if(tempFilename!="") {
    if(setupPlanning(tempFilename)) {
      repaint();
    }
  }
}

bool BIRRTstarWindow::setupPlanning(QString filename) {
  if(mpViz) {
    mpViz->m_PPInfo.loadFromFile(filename);
    openMap(mpViz->m_PPInfo.mMapFullpath);
    if(mpBIRRTstarConfig) {
      mpBIRRTstarConfig->updateDisplay();
    }
    return true;
  }
  return false;
}

void BIRRTstarWindow::onSave() {
  QString tempFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("XML Files (*.xml)"));

  if(tempFilename!="") {
    if(mpViz) {
      mpViz->m_PPInfo.saveToFile(tempFilename);
    }
  }
}

void BIRRTstarWindow::onExport() {
  QString pathFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("Txt Files (*.txt)"));
  if (pathFilename != "") {
    mpViz->m_PPInfo.mPathsOutput = pathFilename;
    exportPaths();
  }
}

bool BIRRTstarWindow::exportPaths() {
  if(mpViz) {
    bool success = false;
    success = mpViz->m_PPInfo.exportPaths(mpViz->m_PPInfo.mPathsOutput);
    success = mpViz->drawPath(mpViz->m_PPInfo.mPathsOutput+".png");
    return success;
  }
  return false;
}

void BIRRTstarWindow::onLoadMap() {
  QString tempFilename = QFileDialog::getOpenFileName(this,
             tr("Open Map File"), "./", tr("Map Files (*.*)"));

  if(tempFilename != "") {
    QFileInfo fileInfo(tempFilename);
    QString filename(fileInfo.fileName());
    mpViz->m_PPInfo.mMapFilename = filename;
    mpViz->m_PPInfo.mMapFullpath = tempFilename;
    qDebug("OPENING ");
    //qDebug(mpViz->m_PPInfo.m_map_filename.toStdString().c_str());
    openMap(mpViz->m_PPInfo.mMapFullpath);
  }
}


bool BIRRTstarWindow::openMap(QString filename) {
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

void BIRRTstarWindow::onLoadObj() {
  mpBIRRTstarConfig->exec();
  updateTitle();
}

void BIRRTstarWindow::onRun() {
  if (mpViz->m_PPInfo.mMapWidth <= 0 || mpViz->m_PPInfo.mMapHeight <= 0) {
    QMessageBox msgBox;
    msgBox.setText("Map is not initialized.");
    msgBox.exec();
    return;
  }
  if(mpViz->m_PPInfo.m_start.x()<0 || mpViz->m_PPInfo.m_start.y()<0) {
    QMessageBox msgBox;
    msgBox.setText("Start is not set.");
    msgBox.exec();
    return;
  }
  if(mpViz->m_PPInfo.m_goal.x()<0 || mpViz->m_PPInfo.m_goal.y()<0) {
    QMessageBox msgBox;
    msgBox.setText("Goal is not set.");
    msgBox.exec();
    return;
  }

  planPath();
  repaint();
}

void BIRRTstarWindow::onExportGrammar() {

  QString grammarFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("DOT Files (*.dot)"));
  if( mpReferenceFrameSet ) {
    StringGrammar* p_grammar = mpReferenceFrameSet->getStringGrammar( mpViz->m_PPInfo.m_start.x(), mpViz->m_PPInfo.m_start.y(), mpViz->m_PPInfo.m_goal.x(), mpViz->m_PPInfo.m_goal.y() );
    if( p_grammar ) {
      p_grammar->output( grammarFilename.toStdString() );
      //mlrrts::ExpandingTree tree;
      //tree.init( p_grammar );
      //QString expandingTreeFilename = grammarFilename.replace(".dot","") + "-exp.dot";
      //tree.output( expandingTreeFilename.toStdString() );
    }
  }
}

void BIRRTstarWindow::planPath() {

  if(mpBIRRTstar==NULL) {

      if (mpViz) {
        for( std::vector<Path*>::iterator it = mpViz->m_PPInfo.mpFoundPaths.begin();
             it != mpViz->m_PPInfo.mpFoundPaths.end(); it ++ ) {
          Path * p_path = (*it);
          delete p_path;
          p_path = NULL;
        }
        mpViz->m_PPInfo.mpFoundPaths.clear();
      }

      mpViz->m_PPInfo.initFuncParam();
      QString msg = "INIT RRTstar ... \n";
      msg += "SegmentLen( " + QString::number(mpViz->m_PPInfo.mSegmentLength) + " ) \n";
      msg += "MaxIterationNum( " + QString::number(mpViz->m_PPInfo.mMaxIterationNum) + " ) \n";
      qDebug() << msg;

      mpBIRRTstar = new BIRRTstar(mpMap->width(), mpMap->height(), mpViz->m_PPInfo.mSegmentLength);
      mpBIRRTstar->setReferenceFrames( mpReferenceFrameSet );
      mpBIRRTstar->setRunType( mpViz->m_PPInfo.mRunType );
      POS2D start(mpViz->m_PPInfo.m_start.x(), mpViz->m_PPInfo.m_start.y());
      POS2D goal(mpViz->m_PPInfo.m_goal.x(), mpViz->m_PPInfo.m_goal.y());

      mpBIRRTstar->init(start, goal, mpViz->m_PPInfo.mpFunc, mpViz->m_PPInfo.mCostDistribution, mpViz->m_PPInfo.mGrammarType);
      mpViz->m_PPInfo.getObstacleInfo(mpBIRRTstar->getMapInfo());
      mpViz->setTree(mpBIRRTstar);
  }

  mpViz->setFinishedPlanning( false );
  //mpBIRRTstar->dump_distribution("dist.txt");
  while(mpBIRRTstar->getCurrentIteration() <= mpViz->m_PPInfo.mMaxIterationNum) {
    QString msg = "CurrentIteration " + QString::number(mpBIRRTstar->getCurrentIteration()) + " ";
    mpBIRRTstar->extend();
    msg += QString::number(mpBIRRTstar->getStringClassMgr()->getStringClasses().size());
    qDebug() << msg;

    QApplication::processEvents();

    updateStatus();
    repaint();
  }

  qDebug() << "START MERGE ";
  mpBIRRTstar->getStringClassMgr()->merge();
  qDebug() << "END MERGE ";

  //Path* path = mpBIRRTstar->find_path();
  std::vector<Path*> p_paths = mpBIRRTstar->getPaths();
  mpViz->m_PPInfo.loadPaths(p_paths);

  mpViz->setFinishedPlanning( true );
  repaint();
}

void BIRRTstarWindow::onAddStart() {
  mpViz->m_PPInfo.m_start = mCursorPoint;
  repaint();
}

void BIRRTstarWindow::onAddGoal() {
  mpViz->m_PPInfo.m_goal = mCursorPoint;
  repaint();
}

void BIRRTstarWindow::onSaveScreen() {
  QString tempFilename = QFileDialog::getSaveFileName(this, tr("Save PNG File"), "./", tr("PNG Files (*.png)"));
  mpViz->saveCurrentViz( tempFilename );
}

void BIRRTstarWindow::contextMenuRequested(QPoint point) {
  mCursorPoint = point;
  mpContextMenu->popup(mapToGlobal(point));
}

void BIRRTstarWindow::updateTitle() {
  QString title = mpViz->m_PPInfo.mMapFilename;
  setWindowTitle(title);
}

void BIRRTstarWindow::updateStatus() {
  if(mpViz==NULL) {
    return;
  }
  if(mpStatusProgressBar) {
    if(mpBIRRTstar) {
      mpStatusProgressBar->setMinimum(0);
      mpStatusProgressBar->setMaximum(mpViz->m_PPInfo.mMaxIterationNum);
      if(mpBIRRTstar) {
        mpStatusProgressBar->setValue(mpBIRRTstar->getCurrentIteration());
      }
      else {
        mpStatusProgressBar->setValue(0);
      }
    }
  }
  if(mpStatusLabel) {
    QString status = "";
    if (mpViz->getFinishedPlanning() == false) {
      status += QString::fromStdString(mpViz->getRegionName());
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
  repaint();
}

void BIRRTstarWindow::keyPressEvent(QKeyEvent *event) {
  if ( event->key() == Qt::Key_R  ) {
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
      if(mpViz->showRegions() == true) {
        mpViz->setShowRegions( false );
      }
      else {
        mpViz->setShowRegions( true );
      }
      updateStatus();
      repaint();
    }
  }
  else if ( event->key() == Qt::Key_T ) {
    if(mpViz) {
      mpViz->switchTreeShowType();
      std::cout << "TREE DISP " <<mpViz->getTreeShowType();
    }
    updateStatus();
    repaint();
  }
  else if ( event->key() == Qt::Key_I ) {
    if(mpViz) {
      if( mpViz->getDrawedPoints().size() > 1 ) {
        mpViz->importStringConstraint( mpViz->getDrawedPoints(), mpViz->m_PPInfo.mGrammarType );
        mpViz->setShowDrawedPoints(false);
      }
    }
    updateStatus();
    repaint();
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
      mpViz->prevRegion();
      updateStatus();
      repaint();
    }
  }
  else if ( event->key() == Qt::Key_PageDown ) {
    if(mpViz) {
      mpViz->nextRegion();
      updateStatus();
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

void BIRRTstarWindow::onExportAllSimpleStrings() {

  QString stringFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("TXT Files (*.txt)"));
  if( mpReferenceFrameSet ) {
    std::vector< std::vector< std::string > > simple_strings;
    StringGrammar* p_grammar = mpReferenceFrameSet->getStringGrammar( mpViz->m_PPInfo.m_start.x(), mpViz->m_PPInfo.m_start.y(), mpViz->m_PPInfo.m_goal.x(), mpViz->m_PPInfo.m_goal.y() );
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

void BIRRTstarWindow::onReset() {
  if(mpBIRRTstar) {
    delete mpBIRRTstar;
    mpBIRRTstar = NULL;
  }
  mpViz->reset();
  updateStatus();
  repaint();
}

void BIRRTstarWindow::onExportStringClassHist() {
  QString pathFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("Txt Files (*.txt)"));
  if (pathFilename != "") {
    if(mpBIRRTstar) {    
      mpBIRRTstar->getStringClassMgr()->dumpHistoricalData( pathFilename.toStdString() );
    }
  }
}

} // harrts

} // topologyPathPlanning
