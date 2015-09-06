#ifndef HARRTSTAR_VIZ_H_
#define HARRTSTAR_VIZ_H_

#include <QLabel>

#include "HARRTstar.h"
#include "path_planning_info.h"

class HARRTstarViz : public QLabel
{
    Q_OBJECT
public:
    explicit HARRTstarViz(QWidget *parent = 0);
    void setTree(HARRTstar* p_tree);
    bool drawPath(QString filename);

    PathPlanningInfo m_PPInfo;
signals:
    
public slots:

private:
    void drawPathOnMap(QPixmap& map);
    HARRTstar* mp_tree;

private slots:
    void paintEvent(QPaintEvent * e);
};

#endif // HARRTSTAR_VIZ_H_
