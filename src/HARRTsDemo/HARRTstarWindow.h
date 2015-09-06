#ifndef HARRT_MAINWINDOW_H
#define HARRT_MAINWINDOW_H

#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QPixmap>
#include <QProgressBar>
#include "HARRTstarViz.h"

class HARRTConfigDialog;

class HARRTWindow : public QMainWindow {
    Q_OBJECT
    
public:
    HARRTWindow(QWidget *parent = 0);
    ~HARRTWindow();

    bool exportPaths();
    void planPath();
    bool setupPlanning(QString filename);
    HARRTstarViz * mpViz;

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

    HARRTConfigDialog* mpHARRTConfigDialog;
    HARRTstar*         mpHARRTstar;


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
