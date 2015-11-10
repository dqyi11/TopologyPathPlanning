#include <QFile>
#include <QTextStream>
#include <QFileDialog>
#include <QMessageBox>
#include <QtDebug>
#include <QKeyEvent>
#include <QStatusBar>
#include "HARRTstarConfig.h"
#include "HARRTstarWindow.h"
#include "img_load_util.h"

using namespace homotopy;
using namespace harrts;

HARRTstarWindow::HARRTstarWindow(QWidget *parent)
    : QMainWindow(parent) {
    mpViz = new HARRTstarViz();

    createActions();
    createMenuBar();

    mpMap = NULL;
    mpHARRTstar = NULL;
    mpReferenceFrameSet = NULL;

    mpHARRTstarConfig = new HARRTstarConfig(this);
    mpHARRTstarConfig->hide();

    setCentralWidget(mpViz);

    mpStatusLabel = new QLabel();
    mpStatusLabel->setFixedWidth(120);
    mpStatusProgressBar = new QProgressBar();
    
    statusBar()->addWidget(mpStatusLabel);
    statusBar()->addWidget(mpStatusProgressBar);
    updateTitle();
}

HARRTstarWindow::~HARRTstarWindow() {
    if(mpHARRTstarConfig) {
        delete mpHARRTstarConfig;
        mpHARRTstarConfig = NULL;
    }
    if(mpHARRTstar) {
        delete mpHARRTstar;
        mpHARRTstar = NULL;
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

void HARRTstarWindow::createMenuBar() {
    mpFileMenu = menuBar()->addMenu("&File");
    mpFileMenu->addAction(mpOpenAction);
    mpFileMenu->addAction(mpSaveAction);
    mpFileMenu->addAction(mpExportAction);

    mpEditMenu = menuBar()->addMenu("&Edit");
    mpEditMenu->addAction(mpLoadMapAction);
    mpEditMenu->addAction(mpLoadObjAction);
    mpEditMenu->addAction(mpRunAction);

    mpToolMenu = menuBar()->addMenu("&Tool");
    mpToolMenu->addAction(mpSaveScreenAction);
    mpToolMenu->addAction(mpExportGrammarGraphAction);
    mpToolMenu->addAction(mpExportAllSimpleStringsAction);

    mpContextMenu = new QMenu();
    setContextMenuPolicy(Qt::CustomContextMenu);

    mpContextMenu->addAction(mpAddStartAction);
    mpContextMenu->addAction(mpAddGoalAction);

}

void HARRTstarWindow::createActions() {
    mpOpenAction = new QAction("Open", this);
    mpSaveAction = new QAction("Save", this);
    mpExportAction = new QAction("Export", this);
    mpLoadMapAction = new QAction("Load Map", this);
    mpLoadObjAction = new QAction("Config Objective", this);
    mpRunAction = new QAction("Run", this);

    connect(mpOpenAction, SIGNAL(triggered()), this, SLOT(onOpen()));
    connect(mpSaveAction, SIGNAL(triggered()), this, SLOT(onSave()));
    connect(mpExportAction, SIGNAL(triggered()), this, SLOT(onExport()));
    connect(mpLoadMapAction, SIGNAL(triggered()), this, SLOT(onLoadMap()));
    connect(mpLoadObjAction, SIGNAL(triggered()), this, SLOT(onLoadObj()));
    connect(mpRunAction, SIGNAL(triggered()), this, SLOT(onRun()));

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

void HARRTstarWindow::onOpen() {
    QString tempFilename = QFileDialog::getOpenFileName(this,
             tr("Open File"), "./", tr("XML Files (*.xml)"));

    if(setupPlanning(tempFilename)) {
        repaint();
    }
}

bool HARRTstarWindow::setupPlanning(QString filename) {
    if(mpViz) {
        mpViz->m_PPInfo.load_from_file(filename);
        openMap(mpViz->m_PPInfo.m_map_fullpath);
        if(mpHARRTstarConfig) {
            mpHARRTstarConfig->updateDisplay();
        }
        return true;
    }
    return false;
}

void HARRTstarWindow::onSave() {
    QString tempFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("XML Files (*.xml)"));

    if(mpViz) {
        mpViz->m_PPInfo.save_to_file(tempFilename);
    }
}

void HARRTstarWindow::onExport() {
    QString pathFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("Txt Files (*.txt)"));
    if (pathFilename != "") {
        mpViz->m_PPInfo.m_paths_output = pathFilename;
        exportPaths();
    }
}

bool HARRTstarWindow::exportPaths() {
    if(mpViz) {
        bool success = false;
        success = mpViz->m_PPInfo.export_paths(mpViz->m_PPInfo.m_paths_output);
        success = mpViz->drawPath(mpViz->m_PPInfo.m_paths_output+".png");
        return success;
    }
    return false;
}

void HARRTstarWindow::onLoadMap() {
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


bool HARRTstarWindow::openMap(QString filename) {
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
        mpViz->setReferenceFrameSet( mpReferenceFrameSet );        
        return true;
    }
    return false;
}

void HARRTstarWindow::onLoadObj() {
    mpHARRTstarConfig->exec();
    updateTitle();
}

void HARRTstarWindow::onRun() {
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

void HARRTstarWindow::onExportGrammar() {

    QString grammarFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("DOT Files (*.dot)"));
    if( mpReferenceFrameSet ) {
        StringGrammar* p_grammar = mpReferenceFrameSet->get_string_grammar( mpViz->m_PPInfo.m_start.x(), mpViz->m_PPInfo.m_start.y(), mpViz->m_PPInfo.m_goal.x(), mpViz->m_PPInfo.m_goal.y() );
        if( p_grammar ) {
            p_grammar->output( grammarFilename.toStdString() );
        }
    }
}

void HARRTstarWindow::planPath() {
    if(mpHARRTstar) {
        delete mpHARRTstar;
        mpHARRTstar = NULL;
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

    mpViz->m_PPInfo.init_func_param();
    QString msg = "INIT RRTstar ... \n";
    msg += "SegmentLen( " + QString::number(mpViz->m_PPInfo.m_segment_length) + " ) \n";
    msg += "MaxIterationNum( " + QString::number(mpViz->m_PPInfo.m_max_iteration_num) + " ) \n";
    qDebug() << msg;

    mpHARRTstar = new HARRTstar(mpMap->width(), mpMap->height(), mpViz->m_PPInfo.m_segment_length);
    mpHARRTstar->set_reference_frames( mpReferenceFrameSet );
    mpHARRTstar->set_run_type( mpViz->m_PPInfo.m_run_type );
    POS2D start(mpViz->m_PPInfo.m_start.x(), mpViz->m_PPInfo.m_start.y());
    POS2D goal(mpViz->m_PPInfo.m_goal.x(), mpViz->m_PPInfo.m_goal.y());
    
    mpHARRTstar->init(start, goal, mpViz->m_PPInfo.mp_func, mpViz->m_PPInfo.mCostDistribution, mpViz->m_PPInfo.m_grammar_type);
    mpViz->m_PPInfo.get_obstacle_info(mpHARRTstar->get_map_info());
    mpViz->setTree(mpHARRTstar);
    mpViz->set_finished_planning( false );
    

    //mpHARRTstar->dump_distribution("dist.txt");
    while(mpHARRTstar->get_current_iteration() <= mpViz->m_PPInfo.m_max_iteration_num) {
        QString msg = "CurrentIteration " + QString::number(mpHARRTstar->get_current_iteration()) + " ";
        mpHARRTstar->extend();
        msg += QString::number(mpHARRTstar->get_string_class_mgr()->get_string_classes().size()); 
        qDebug() << msg;

        updateStatus();
        repaint();
    }
    qDebug() << "START MERGE ";
    mpHARRTstar->get_string_class_mgr()->merge();
    qDebug() << "END MERGE ";
    //Path* path = mpHARRTstar->find_path();
    std::vector<Path*> p_paths = mpHARRTstar->get_paths();
    mpViz->m_PPInfo.load_paths(p_paths);
    mpViz->set_finished_planning( true );
    repaint();
}

void HARRTstarWindow::onAddStart() {
    mpViz->m_PPInfo.m_start = mCursorPoint;
    repaint();
}

void HARRTstarWindow::onAddGoal() {
    mpViz->m_PPInfo.m_goal = mCursorPoint;
    repaint();
}

void HARRTstarWindow::onSaveScreen() {
    QString tempFilename = QFileDialog::getSaveFileName(this, tr("Save PNG File"), "./", tr("PNG Files (*.png)"));
    mpViz->saveCurrentViz( tempFilename );
}

void HARRTstarWindow::contextMenuRequested(QPoint point) {
    mCursorPoint = point;
    mpContextMenu->popup(mapToGlobal(point));
}

void HARRTstarWindow::updateTitle() {
    QString title = mpViz->m_PPInfo.m_map_filename;
    setWindowTitle(title);
}

void HARRTstarWindow::updateStatus() {
    if(mpViz==NULL) {
        return;
    }
    if(mpStatusProgressBar) {
        if(mpHARRTstar) {
            mpStatusProgressBar->setMinimum(0);
            mpStatusProgressBar->setMaximum(mpViz->m_PPInfo.m_max_iteration_num);
            mpStatusProgressBar->setValue(mpHARRTstar->get_current_iteration());
        }
    }
    if(mpStatusLabel) {
        QString status = "";
        if (mpViz->get_finished_planning() == false) {
            status += QString::fromStdString(mpViz->get_region_name());
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
    repaint();
}

void HARRTstarWindow::keyPressEvent(QKeyEvent *event) {
   if ( event->key() == Qt::Key_R  ) {
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
           if(mpViz->show_regions() == true) {
               mpViz->set_show_regions( false );
           }
           else {
               mpViz->set_show_regions( true );
           }
           updateStatus();
           repaint();
       }
   }
   else if ( event->key() == Qt::Key_T ) {
       if(mpViz) {
           mpViz->switch_tree_show_type();
           std::cout << "TREE DISP " <<mpViz->get_tree_show_type();
       }
       updateStatus();
       repaint();
   }
   else if ( event->key() == Qt::Key_I ) {
       if(mpViz) {
           if( mpViz->get_drawed_points().size() > 1 ) {
              mpViz->import_string_constraint( mpViz->get_drawed_points(), mpViz->m_PPInfo.m_grammar_type );
              mpViz->set_show_drawed_points(false);
           }
       }
       updateStatus();
       repaint();
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
           mpViz->prev_region();
           updateStatus();
           repaint();
       }
   }
   else if ( event->key() == Qt::Key_PageDown ) {
       if(mpViz) {
           mpViz->next_region();
           updateStatus();
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

void HARRTstarWindow::onExportAllSimpleStrings() {

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
                for( int i=0; i < ids.size(); i ++) {
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
