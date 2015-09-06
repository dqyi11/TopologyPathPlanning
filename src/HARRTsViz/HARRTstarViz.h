#ifndef HARRTSTAR_VIZ_H_
#define HARRTSTAR_VIZ_H_

#include <QLabel>

#include "HARRTstar.h"
#include "path_planning_info.h"
#include "reference_frames.h"

class HARRTstarViz : public QLabel
{
    Q_OBJECT
public:
    explicit HARRTstarViz(QWidget *parent = 0);
    void setTree(HARRTstar* p_tree);
    void setReferenceFrameSet(ReferenceFrameSet* p_rf);
    bool drawPath(QString filename);

    PathPlanningInfo m_PPInfo;
signals:
    
public slots:

private:
    void drawPathOnMap(QPixmap& map);
    HARRTstar*         mp_tree;
    ReferenceFrameSet* mp_reference_frames;

private slots:
    void paintEvent(QPaintEvent * e);
};

#endif // HARRTSTAR_VIZ_H_
