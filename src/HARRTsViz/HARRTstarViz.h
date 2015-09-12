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
    void set_show_regions( bool show );
    bool show_reference_frames() { return m_show_reference_frames; }
    bool show_regions() { return m_show_regions; }
    void prev_reference_frame();
    void next_reference_frame();
    void prev_found_path();
    void next_found_path();
    int  get_reference_frame_index() { return m_reference_frame_index; }
    std::string get_reference_frame_name();
    QString generate_string();

    PathPlanningInfo m_PPInfo;

signals:
    
public slots:

private:
    void drawPathOnMap(QPixmap& map);
    HARRTstar*          mp_tree;
    ReferenceFrameSet*  mp_reference_frames;
    bool                m_show_reference_frames;
    bool                m_show_regions;
    int                 m_reference_frame_index;
    int                 m_found_path_index;
    std::vector<QColor> m_colors;

private slots:
    void paintEvent(QPaintEvent * e);
};

#endif // HARRTSTAR_VIZ_H_
