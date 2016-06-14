#include <QFile>
#include <QTextStream>
#include <QFileDialog>
#include <QMessageBox>
#include <QtDebug>
#include <QKeyEvent>
#include <QStatusBar>
#include "mlrrtstar_config.h"
#include "mlrrtstar_window.h"
#include "img_load_util.h"
#include "expanding_tree.h"

using namespace std;
using namespace homotopy;
using namespace mlrrts;

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
  mpLoadMapAction = new QAction("Load Map", this);
  mpLoadObjAction = new QAction("Config Objective", this);
  mpRunAction = new QAction("Run", this);
  mpResetAction = new QAction("Reset", this);

  connect(mpOpenAction, SIGNAL(triggered()), this, SLOT(onOpen()));
  connect(mpSaveAction, SIGNAL(triggered()), this, SLOT(onSave()));
  connect(mpExportAction, SIGNAL(triggered()), this, SLOT(onExport()));
  connect(mpExportPathAction, SIGNAL(triggered()), this, SLOT(onExportPath()));

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
    success = mpViz->m_PPInfo.export_paths(mpViz->m_PPInfo.m_paths_output);
    success = mpViz->draw_path(mpViz->m_PPInfo.m_paths_output+".png");
    return success;
  }
  return false;
}

bool MLRRTstarWindow::setupPlanning(QString filename) {
  if (mpViz) {
    mpViz->m_PPInfo.load_from_file(filename);
    openMap(mpViz->m_PPInfo.m_map_fullpath);
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
      if( mpViz->m_PPInfo.m_min_dist_enabled == false ) {
        if(mpViz->m_PPInfo.mp_obj==NULL) {
          mpViz->m_PPInfo.init_obj_pixmap();
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
    mpViz->m_PPInfo.save_to_file(tempFilename);
  }
}

void MLRRTstarWindow::onExport() {
  QString pathFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("Txt Files (*.txt)"));
  if (pathFilename != "") {
    mpViz->m_PPInfo.m_paths_output = pathFilename;
    exportPaths();
  }
}

void MLRRTstarWindow::onExportPath() {

  QString pathFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("Txt Files (*.txt)"));
  if (pathFilename != "") {

    Path* p_viz_path = mpViz->get_viz_path();
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
  mpViz->m_PPInfo.m_map_filename = filename;
  mpViz->m_PPInfo.m_map_fullpath = tempFilename;
  qDebug("OPENING ");
  //qDebug(mpViz->m_PPInfo.m_map_filename.toStdString().c_str());

  openMap(mpViz->m_PPInfo.m_map_fullpath);
}


bool MLRRTstarWindow::openMap(QString filename) {
  if(mpMap) {
    delete mpMap;
    mpMap = NULL;
  }
  mpMap = new QPixmap(filename);
  if(mpMap) {
    mpViz->m_PPInfo.m_map_width = mpMap->width();
    mpViz->m_PPInfo.m_map_height = mpMap->height();
    mpViz->setPixmap(*mpMap);
    updateTitle();

    int map_width = 0, map_height = 0;
    std::vector< std::vector< Point2D > > obstacles;
    mpReferenceFrameSet = new ReferenceFrameSet();
    load_map_info( filename.toStdString(), map_width, map_height, obstacles );
    mpReferenceFrameSet->init( map_width, map_height, obstacles );
    mpViz->set_reference_frame_set( mpReferenceFrameSet );        
    return true;
  }
  return false;
}

void MLRRTstarWindow::onLoadObj() {
  mpMLRRTstarConfig->exec();
  updateTitle();
}

void MLRRTstarWindow::onRun() {
  if (mpViz->m_PPInfo.m_map_width <= 0 || mpViz->m_PPInfo.m_map_height <= 0) {
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

void MLRRTstarWindow::onExportGrammar() {

  QString grammarFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("DOT Files (*.dot)"));
  if( mpReferenceFrameSet ) {
    StringGrammar* p_grammar = mpReferenceFrameSet->get_string_grammar( mpViz->m_PPInfo.m_start.x(), mpViz->m_PPInfo.m_start.y(), mpViz->m_PPInfo.m_goal.x(), mpViz->m_PPInfo.m_goal.y() );
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
      mpViz->m_PPInfo.init_func_param();
      QString msg = "INIT RRTstar ... \n";
      msg += "SegmentLen( " + QString::number(mpViz->m_PPInfo.m_segment_length) + " ) \n";
      msg += "MaxIterationNum( " + QString::number(mpViz->m_PPInfo.m_max_iteration_num) + " ) \n";
      qDebug() << msg;

      mpMLRRTstar = new MLRRTstar(mpMap->width(), mpMap->height(), mpViz->m_PPInfo.m_segment_length);
      mpMLRRTstar->set_reference_frames( mpReferenceFrameSet );
      if( mpViz->m_PPInfo.m_homotopic_enforcement ) {
        mpMLRRTstar->set_homotopic_enforcement( true );
      }
      else {
        mpMLRRTstar->set_homotopic_enforcement( false );
      }
      POS2D start(mpViz->m_PPInfo.m_start.x(), mpViz->m_PPInfo.m_start.y());
      POS2D goal(mpViz->m_PPInfo.m_goal.x(), mpViz->m_PPInfo.m_goal.y());

      mpMLRRTstar->init(start, goal, mpViz->m_PPInfo.mp_func, mpViz->m_PPInfo.m_cost_distribution, mpViz->m_PPInfo.m_grammar_type);
      mpViz->m_PPInfo.get_obstacle_info(mpMLRRTstar->get_map_info());
      mpViz->set_tree(mpMLRRTstar);

      mpMLRRTstar->get_expanding_tree_mgr()->get_expanding_tree()->output( "output.dot" );
      mpMLRRTstar->get_expanding_tree_mgr()->get_expanding_tree()->print();
      mpMLRRTstar->get_expanding_tree_mgr()->export_subregion_mgrs("subregion_mgrs.txt");
  }

  mpViz->set_finished_planning( false );

  while( ( false == mpViz->is_finished_planning() )
         && mpMLRRTstar->get_current_iteration() <= mpViz->m_PPInfo.m_max_iteration_num) {
    QString msg = "CurrentIteration " + QString::number(mpMLRRTstar->get_current_iteration()) + " ";
    mpMLRRTstar->extend();
    msg += QString::number(mpMLRRTstar->get_expanding_tree_mgr()->get_string_classes().size()); 
    qDebug() << msg;

    updateStatus();
    repaint();
  }

  qDebug() << "START MERGE ";
  //mpMLRRTstar->get_string_class_mgr()->merge();
  qDebug() << "END MERGE ";
  //Path* path = mpMLRRTstar->find_path();
  std::vector<Path*> p_paths = mpMLRRTstar->get_paths();
  mpViz->m_PPInfo.load_paths(p_paths);

  mpViz->set_finished_planning( true );
  repaint();
}

void MLRRTstarWindow::onAddStart() {
  mpViz->m_PPInfo.m_start = mCursorPoint;
  repaint();
}

void MLRRTstarWindow::onAddGoal() {
  mpViz->m_PPInfo.m_goal = mCursorPoint;
  repaint();
}

void MLRRTstarWindow::onSaveScreen() {
  QString tempFilename = QFileDialog::getSaveFileName(this, tr("Save PNG File"), "./", tr("PNG Files (*.png)"));
  mpViz->save_current_viz( tempFilename );
}

void MLRRTstarWindow::contextMenuRequested(QPoint point) {
  mCursorPoint = point;
  mpContextMenu->popup(mapToGlobal(point));
}

void MLRRTstarWindow::updateTitle() {
  QString title = mpViz->m_PPInfo.m_map_filename;
  setWindowTitle(title);
}

void MLRRTstarWindow::updateStatus() {
  if(mpViz==NULL) {
    return;
  }
  if(mpStatusProgressBar) {
    if(mpMLRRTstar) {
      mpStatusProgressBar->setMinimum(0);
      mpStatusProgressBar->setMaximum(mpViz->m_PPInfo.m_max_iteration_num);
      mpStatusProgressBar->setValue(mpMLRRTstar->get_current_iteration());
    }
  }
  if(mpStatusLabel) {
    QString status = "";
    if (mpViz->is_finished_planning() == false) {
      status += QString::fromStdString(mpViz->get_subregion_name());
      status += " || ";
      status += QString::fromStdString(mpViz->get_reference_frame_name());
    }
    else {
      status += QString::number( mpViz->get_found_path_index() );
      status += " / ";
      status += QString::number( mpViz->m_PPInfo.mp_found_paths.size() );
    }
    mpStatusLabel->setText(status);
  }
  if(mpStringClassLabel) {
    QString status = mpViz->get_string_class_info();
    mpStringClassLabel->setText(status);
  }
  repaint();
}

void MLRRTstarWindow::keyPressEvent(QKeyEvent *event) {
   if ( event->key() == Qt::Key_Escape ) {
     if( mpViz ) {
       mpViz->set_finished_planning( true );
     }
     updateStatus();
     repaint();
   }
   else if ( event->key() == Qt::Key_R  ) {
     if(mpViz) {
       if(mpViz->show_reference_frames() == true) {
         mpViz->set_show_reference_frames( false );
       }
       else {
         mpViz->set_show_reference_frames( true );
       }
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_S ) {
     if(mpViz) {
       if(mpViz->show_subregions() == true) {
         mpViz->set_show_subregions( false );
       }
       else {
         mpViz->set_show_subregions( true );
       }
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_P ) {
     if(mpViz) {
       if(mpViz->show_paths() == true) {
         mpViz->set_show_paths( false );
       }
       else {
         mpViz->set_show_paths( true );
       }
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_I ) {
     if(mpViz) {
       if( mpViz->get_drawed_points().size() > 1 ) {
         mpViz->import_string_constraint( mpViz->get_drawed_points(), mpViz->m_PPInfo.m_grammar_type );
         mpViz->set_show_drawed_points(false);
         mpViz->set_mode( NORMAL );
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
       if( mpViz->get_mode() == NORMAL ) {
         mpViz->set_mode( DRAWING );
       }
       else{
         mpViz->set_mode( NORMAL );
       }
     }
   }
   else if ( event->key() == Qt::Key_Up ) {
     if(mpViz) {
       mpViz->prev_reference_frame();
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_Down ) {
     if(mpViz) {
       mpViz->next_reference_frame();
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_Left ) {
     if(mpViz) {
       mpViz->prev_found_path();
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_Right ) {
     if(mpViz) {
       mpViz->next_found_path();
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_PageUp ) {
     if(mpViz) {
       mpViz->prev_string_class();
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_PageDown ) {
     if(mpViz) {
       mpViz->next_string_class();
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_Q ) {
     if(mpViz) {
       mpViz->prev_exp_node();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_W ) {
     if(mpViz) {
       mpViz->next_exp_node();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_Z ) {
     if(mpViz) {
       mpViz->prev_subregion();
       updateStatus();
       repaint();
     }
   }
   else if ( event->key() == Qt::Key_X ) {
     if(mpViz) {
       mpViz->next_subregion();
       updateStatus();
       repaint();
     }
   }
   
}

void MLRRTstarWindow::onExportAllSimpleStrings() {

  QString stringFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("TXT Files (*.txt)"));
  if( mpReferenceFrameSet ) {
    std::vector< std::vector< std::string > > simple_strings;
    StringGrammar* p_grammar = mpReferenceFrameSet->get_string_grammar( mpViz->m_PPInfo.m_start.x(), mpViz->m_PPInfo.m_start.y(), mpViz->m_PPInfo.m_goal.x(), mpViz->m_PPInfo.m_goal.y() );
    if( p_grammar ) {
      simple_strings = p_grammar->get_all_simple_strings();
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
        if( p_grammar->is_valid_string( ids )) {
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
      for( unsigned int i=0; i < path->m_way_points.size(); i++ ) {
        stream << path->m_way_points[i][0] << " " << path->m_way_points[i][1] << "\n";
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
      for( std::vector<Path*>::iterator it = mpViz->m_PPInfo.mp_found_paths.begin();
           it != mpViz->m_PPInfo.mp_found_paths.end(); it ++ ) {
        Path * p_path = (*it);
        delete p_path;
        p_path = NULL;
      }
      mpViz->m_PPInfo.mp_found_paths.clear();
    }

}
