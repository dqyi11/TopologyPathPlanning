#ifndef TOPOLOGYPATHPLANNING_TARRTS_MLRRTSTARWINDOW_HPP
#define TOPOLOGYPATHPLANNING_TARRTS_MLRRTSTARWINDOW_HPP

#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QPixmap>
#include <QProgressBar>
#include "topologyPathPlanning/homotopy/ReferenceFrames.hpp"
#include "topologyPathPlanning/tarrtsviz/MLRRTstarViz.hpp"

namespace topologyPathPlanning {

namespace tarrts {

  class MLRRTstarConfig;
  
  class MLRRTstarWindow : public QMainWindow {
    Q_OBJECT
  
  public:
    MLRRTstarWindow(QWidget* parent=0);
    ~MLRRTstarWindow();

    bool exportPaths();
    bool exportPath( Path* path, QString filename );
    void planPath();
    bool setupPlanning(QString filename);
    MLRRTstarViz* mpViz;

    void keyPressEvent(QKeyEvent* e);

    void setShowObj( bool show );
    bool getShowObj() { return mShowObj; }

  protected:
    void createMenuBar();
    void createActions();
    bool openMap(QString filename);
    void updateStatus();

  private:
    void updateTitle();

    QMenu*   mpFileMenu;
    QAction* mpOpenAction;
    QAction* mpSaveAction;
    QAction* mpExportAction;
    QAction* mpExportPathAction;

    QMenu*   mpEditMenu;
    QAction* mpLoadMapAction;
    QAction* mpLoadObjAction;
    QAction* mpRunAction;
    QAction* mpResetAction;

    QMenu*   mpToolMenu;
    QAction* mpSaveScreenAction;
    QAction* mpExportGrammarGraphAction;
    QAction* mpExportAllSimpleStringsAction;
    QAction* mpExportStringClassHistAction;

    QMenu*   mpContextMenu;
    QAction* mpAddStartAction;
    QAction* mpAddGoalAction;

    QLabel*       mpStatusLabel;
    QProgressBar* mpStatusProgressBar;
    QLabel*       mpStringClassLabel;

    QPixmap* mpMap;
    QPoint   mCursorPoint;

    MLRRTstarConfig*             mpMLRRTstarConfig;
    MLRRTstar*                   mpMLRRTstar;
    homotopy::ReferenceFrameSet* mpReferenceFrameSet;
    bool mShowObj;

  private slots:
    void contextMenuRequested(QPoint point);
    void onOpen();
    void onSave();
    void onExport();
    void onExportPath();
    void onLoadMap();
    void onLoadObj();
    void onRun();
    void onAddStart();
    void onAddGoal();
    void onSaveScreen();
    void onExportGrammar();
    void onExportAllSimpleStrings();
    void onReset();
    void onExportStringClassHist();
  };

} // tarrts

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TARRTS_MLRRTSTARWINDOW_HPP
