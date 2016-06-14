#ifndef BIRRTSTARMAINWINDOW_H
#define BIRRTSTARMAINWINDOW_H

#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QPixmap>
#include <QProgressBar>
#include "birrtstar_viz.h"
#include "reference_frames.h"

namespace birrts {

  class BIRRTstarConfig;

  class BIRRTstarWindow : public QMainWindow {
    Q_OBJECT
    
  public:
    BIRRTstarWindow(QWidget *parent = 0);
    ~BIRRTstarWindow();

    bool exportPaths();
    void planPath();
    bool setupPlanning(QString filename);
    BIRRTstarViz * mpViz;

    void keyPressEvent(QKeyEvent * e);

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
    QAction* mpExportStringClassHistAction;

    QMenu*   mpEditMenu;
    QAction* mpLoadMapAction;
    QAction* mpLoadObjAction;
    QAction* mpRunAction;
    QAction* mpResetAction;

    QMenu*   mpToolMenu;
    QAction* mpSaveScreenAction;
    QAction* mpExportGrammarGraphAction;
    QAction* mpExportAllSimpleStringsAction;

    QMenu*   mpContextMenu;
    QAction* mpAddStartAction;
    QAction* mpAddGoalAction;

    QLabel*       mpStatusLabel;
    QProgressBar* mpStatusProgressBar;

    QPixmap* mpMap;
    QPoint   mCursorPoint;

    BIRRTstarConfig*   mpBIRRTstarConfig;
    BIRRTstar*         mpBIRRTstar;
    homotopy::ReferenceFrameSet* mpReferenceFrameSet;

  private slots:
    void contextMenuRequested(QPoint point);
    void onOpen();
    void onSave();
    void onExport();
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

}

#endif // BIRRTSTARMAINWINDOW_H
