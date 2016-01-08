#ifndef SPATIAL_INFER_WINDOW_H
#define SPATIAL_INFER_WINDOW_H

#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QPixmap>
#include <QProgressBar>
#include <QMessageBox>
#include "spatialinfer_viz.h"

namespace topology_inference {

  class SpatialInferWindow : public QMainWindow {
    Q_OBJECT
    
  public:
    SpatialInferWindow(QWidget *parent = 0);
    ~SpatialInferWindow();

    SpatialInferViz * mpViz;

  protected:
    void createMenuBar();
    void createActions();
    void updateStatusBar();

    void keyPressEvent(QKeyEvent * e);
  private:
    QMessageBox*  mpMsgBox;
    QMenu*        mpFileMenu;
    QAction*      mpOpenAction;
    QAction*      mpSaveAction;
    QAction*      mpLoadAction;
    QLabel*       mpStatusLabel;

    QAction*      mpAddStartAction;
    QAction*      mpAddGoalAction; 
    QMenu*        mpContextMenu;

    QPoint        mCursorPoint;
  private slots:
    void contextMenuRequested( QPoint point );
    void onOpen();
    void onSave();
    void onLoad();
    void onAddStart();
    void onAddGoal();
  };
}

#endif // SPATIAL_INFER_WINDOW_H
