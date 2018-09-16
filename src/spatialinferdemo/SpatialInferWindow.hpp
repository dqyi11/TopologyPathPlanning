#ifndef TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_SPATIALINFERWINDOW_HPP
#define TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_SPATIALINFERWINDOW_HPP

#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QPixmap>
#include <QProgressBar>
#include <QMessageBox>
#include "topologyPathPlanning/spatialinferviz/SpatialInferViz.hpp"

namespace topologyPathPlanning {

namespace topologyinference {
  
  class SpatialInferConfig;

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
    QMenu*        mpAddMenu;
    QMenu*        mpAddSideofRelationMenu;
    QMenu*        mpManageMenu;
    QAction*      mpOpenAction;
    QAction*      mpSaveAction;
    QAction*      mpLoadAction;
    QLabel*       mpStatusLabel;

    QAction*      mpAddStartAction;
    QAction*      mpAddGoalAction; 
    QMenu*        mpContextMenu;

    QAction*      mpAddInbetweenSpatialRelationAction;
    QAction*      mpAddAvoidSpatialRelationAction;
    QAction*      mpAddLeftofSpatialRelationAction;
    QAction*      mpAddRightofSpatialRelationAction;
    QAction*      mpAddTopofSpatialRelationAction;
    QAction*      mpAddBottomofSpatialRelationAction;

    QAction*      mpShowConfigAction;
    QAction*      mpExecuteAction;

    SpatialInferConfig* mpConfig;

    QPoint        mCursorPoint;
  private slots:
    void contextMenuRequested( QPoint point );
    void onOpen();
    void onSave();
    void onLoad();
    void onAddStart();
    void onAddGoal();

    void onAddInbetweenSpatialRelation();
    void onAddAvoidSpatialRelation();
    void onAddLeftofSpatialRelation();
    void onAddRightofSpatialRelation();
    void onAddTopofSpatialRelation();
    void onAddBottomofSpatialRelation();

    void onShowConfig();
    void onExecute();
  };

} // topologyinference

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_SPATIALINFERWINDOW_HPP
