#ifndef BIRRTSTAR_VIZ_H_
#define BIRRTSTAR_VIZ_H_

#include <QLabel>

#include "BIRRTstar.h"
#include "path_planning_info.h"
#include "reference_frames.h"

namespace harrts {

  typedef enum{
    NONE_TREE_SHOW,
    START_TREE_SHOW,
    GOAL_TREE_SHOW,
    BOTH_TREES_SHOW
  } tree_show_type_t;

  class BIRRTstarViz : public QLabel
  {
    Q_OBJECT
  public:
    explicit BIRRTstarViz(QWidget *parent = 0);
    void setTree(BIRRTstar* p_tree);
    void setReferenceFrameSet(homotopy::ReferenceFrameSet* p_rf);
    bool drawPath(QString filename);
    bool saveCurrentViz(QString filename);

    void set_show_reference_frames( bool show );
    void set_show_regions( bool show );
    void set_finished_planning( bool finished ) { m_finished_planning = finished; }
    bool get_finished_planning() { return m_finished_planning; }

    bool show_reference_frames() { return m_show_reference_frames; }
    bool show_regions() { return m_show_regions; }
    bool is_finished_planning() { return m_finished_planning; }
 
    homotopy::ReferenceFrame* get_selected_reference_frame();
    homotopy::SubRegionSet*   get_selected_subregion_set();
    homotopy::SubRegion*      get_selected_subregion();

    void prev_region();
    void next_region();
    void prev_subregion();
    void next_subregion();

    void prev_reference_frame();
    void next_reference_frame();
    void prev_found_path();
    void next_found_path();
    int  get_reference_frame_index() { return m_reference_frame_index; }
    std::string get_reference_frame_name();
    std::string get_region_name();
    QString generate_string();

    tree_show_type_t get_tree_show_type() { return m_tree_show_type; }
    void switch_tree_show_type();
    void import_string_constraint( std::vector< QPoint > points, homotopy::grammar_type_t type );

    PathPlanningInfo m_PPInfo;
    
    std::vector<QPoint>& get_drawed_points() { return m_drawed_points; }
    void set_show_drawed_points( bool show ) { m_show_points = show; }
    bool get_show_drawed_points() { return m_show_points; }

    int get_found_path_index() { return m_found_path_index; }
  signals:
    
  public slots:

  protected:
    void mousePressEvent( QMouseEvent * event );
    void mouseMoveEvent( QMouseEvent * event );
    void mouseReleaseEvent( QMouseEvent * event );
    
    std::vector<QPoint>  m_drawed_points;
    bool                 m_dragging;
    bool                 m_show_points;

  private:
    void drawPathOnMap(QPixmap& map);
    void drawCurrentViz(QPixmap& map);

    void paint(QPaintDevice * device);
    BIRRTstar*          mp_tree;
    homotopy::ReferenceFrameSet*  mp_reference_frames;
    bool                m_show_reference_frames;
    bool                m_show_regions;
    bool                m_finished_planning;
    int                 m_reference_frame_index;
    int                 m_found_path_index;
    int                 m_region_index;
    int                 m_subregion_index;
    std::vector<QColor> m_colors;
    tree_show_type_t    m_tree_show_type;

  private slots:
    void paintEvent(QPaintEvent * e);
  };

}

#endif // BIRRTSTAR_VIZ_H_
