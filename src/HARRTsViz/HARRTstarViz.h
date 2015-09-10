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
    void set_show_reference_frames( bool show );
    bool show_reference_frames() { return m_show_reference_frames; }
    void prev_reference_frame();
    void next_reference_frame();
    void prev_found_path();
    void next_found_path();
    int  get_reference_frame_index() { return m_reference_frame_index; }
    std::string get_reference_frame_name();

    PathPlanningInfo m_PPInfo;

signals:
    
public slots:

private:
    void drawPathOnMap(QPixmap& map);
    HARRTstar*         mp_tree;
    ReferenceFrameSet* mp_reference_frames;
    bool               m_show_reference_frames;
    int                m_reference_frame_index;
    int                m_found_path_index;

private slots:
    void paintEvent(QPaintEvent * e);
};

#endif // HARRTSTAR_VIZ_H_
