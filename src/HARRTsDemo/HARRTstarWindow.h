#ifndef HARRT_MAINWINDOW_H
#define HARRT_MAINWINDOW_H

#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QPixmap>
#include <QProgressBar>
#include "HARRTstarViz.h"
#include "reference_frames.h"

class HARRTstarConfig;

class HARRTstarWindow : public QMainWindow {
    Q_OBJECT
    
public:
    HARRTstarWindow(QWidget *parent = 0);
    ~HARRTstarWindow();

    bool exportPaths();
    void planPath();
    bool setupPlanning(QString filename);
    HARRTstarViz * mpViz;

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

    QMenu*   mpEditMenu;
    QAction* mpLoadMapAction;
    QAction* mpLoadObjAction;
    QAction* mpRunAction;

    QMenu*   mpContextMenu;
    QAction* mpAddStartAction;
    QAction* mpAddGoalAction;

    QLabel*       mpStatusLabel;
    QProgressBar* mpStatusProgressBar;

    QPixmap* mpMap;
    QPoint   mCursorPoint;

    HARRTstarConfig*   mpHARRTstarConfig;
    HARRTstar*         mpHARRTstar;
    ReferenceFrameSet* mpReferenceFrameSet;

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
};

#endif // HARRT_MAINWINDOW_H
