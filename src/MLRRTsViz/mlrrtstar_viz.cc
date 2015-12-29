#include <QtGui>

#include "mlrrtstar_viz.h"
#include "mlviz_util.h"

#define TEXT_COLOR        QColor(0, 0, 0)
#define TREE_COLOR        QColor(160,160,0)
#define TREE_COLOR_ALPHA  QColor(160,160,0,100)
#define START_COLOR             QColor(255,0,0)
#define GOAL_COLOR              QColor(0,0,255)
#define REFERENCE_FRAME_COLOR   QColor(0,255,0)
#define SUBREGION_COLOR         QColor(204,229,255)
#define PATH_COLOR              QColor(255,153,21)
#define DRAWING_LINE_COLOR      QColor(153,76,0)
#define TREE_WIDTH              1
#define LINE_WIDTH              2
#define PATH_WIDTH              4
#define POINT_WIDTH             8
#define TREE_OPACITY            0.4

using namespace std;
using namespace homotopy;
using namespace mlrrts;

MLRRTstarViz::MLRRTstarViz( QWidget * parent ) : QLabel(parent) {

  mp_tree = NULL;
  m_show_reference_frames = false;
  m_show_subregions = false;
  m_show_paths = false;
  m_finished_planning = false;
  m_reference_frame_index = -1;
  m_found_path_index = -1;
  m_subregion_index = -1;
  m_string_class_index = -1;
  mp_reference_frames = NULL;
  m_show_points = false;
  m_mode = NORMAL;
  m_item_selected_name = "";
}

void MLRRTstarViz::set_tree( MLRRTstar* p_tree ) {
  mp_tree = p_tree;
}

void MLRRTstarViz::set_reference_frame_set( ReferenceFrameSet* p_rf ) {
  mp_reference_frames = p_rf;
  updateVizSubregions();
  updateVizReferenceFrames();
}

void MLRRTstarViz::updateVizSubregions() {

  m_viz_subregions.clear();
  if( m_string_class_index < 0 ) {
    for( unsigned int i = 0; i < mp_reference_frames->get_world_map()->get_subregion_set().size(); i++) {
      SubRegionSet* p_subregion_set = mp_reference_frames->get_world_map()->get_subregion_set()[i];
      if(p_subregion_set) {
        for( unsigned int j = 0; j < p_subregion_set->m_subregions.size(); j ++ ) {
          SubRegion* p_subreg = p_subregion_set->m_subregions[j];
          m_viz_subregions.push_back( p_subreg );
        }
      }
    }
  } 
  else {
    StringClass* p_str_cls = mp_tree->get_expanding_tree_mgr()->get_string_classes()[ m_string_class_index ];
    if( p_str_cls ) {
      for( vector<ExpandingNode*>::iterator it_exp = p_str_cls->mp_exp_nodes.begin();
           it_exp != p_str_cls->mp_exp_nodes.end(); it_exp++ ) {
        ExpandingNode* p_exp_node = (*it_exp);
        if ( p_exp_node ) {
          SubRegion* p_subreg = p_exp_node->mp_subregion;
          if ( p_subreg ) {
            m_viz_subregions.push_back( p_subreg );
          }
        }
      }
    }
  }
  m_subregion_index = -1;
}

void MLRRTstarViz::updateVizReferenceFrames() {

  m_viz_reference_frames.clear();
  if( m_string_class_index < 0 ) {
    for( int i = 0; i < mp_reference_frames->get_reference_frames().size(); i ++ ) { 
      ReferenceFrame* p_rf = mp_reference_frames->get_reference_frames()[i];
      m_viz_reference_frames.push_back( p_rf );
    }
  }
  else {
    StringClass* p_str_cls = mp_tree->get_expanding_tree_mgr()->get_string_classes()[ m_string_class_index ];
    if( p_str_cls ) {
      for( vector<ReferenceFrame*>::iterator it_rf = p_str_cls->mp_reference_frames.begin();
           it_rf != p_str_cls->mp_reference_frames.end(); it_rf++ ) {
        ReferenceFrame* p_rf = (*it_rf); 
        m_viz_reference_frames.push_back( p_rf );
      }
    }
  }  
  m_reference_frame_index = -1;
}

void MLRRTstarViz::paint( QPaintDevice* device ) {

  /* DRAW SUB REGION */
  if(m_show_subregions) {

    QPainter rg_painter(device);
    rg_painter.setRenderHint(QPainter::Antialiasing);
    QBrush rg_brush( SUBREGION_COLOR );
    rg_painter.setPen(Qt::NoPen);
    if( m_subregion_index < 0 ) {
      for( unsigned int i = 0; i < m_viz_subregions.size(); i ++ ) {
        SubRegion* p_subreg = m_viz_subregions[i];
        if(p_subreg) {
          QPolygon poly;
          for( unsigned int k = 0; k < p_subreg->m_points.size(); k++) {
            poly << toQPoint( p_subreg->m_points[k] );
          }
          QPainterPath tmpPath;
          tmpPath.addPolygon(poly);
          rg_painter.fillPath(tmpPath, rg_brush);
        }
      }
    }
    else {
      SubRegion* p_subreg = m_viz_subregions[m_subregion_index];
      if(p_subreg) {
        QPolygon poly;
        for( unsigned int k = 0; k < p_subreg->m_points.size(); k++) {
          poly << toQPoint( p_subreg->m_points[k] );
        }
        QPainterPath tmpPath;
        tmpPath.addPolygon(poly);
        rg_painter.fillPath(tmpPath, rg_brush);
      } 
    }
    rg_painter.end();
  }

  /* DRAW TREE */
  if( mp_tree ) {
    QPainter tree_painter(device);
    QPen tree_paintpen;
    if(m_finished_planning) {
      tree_paintpen.setColor(TREE_COLOR_ALPHA);
    }
    else{
      tree_paintpen.setColor(TREE_COLOR);
    }
    tree_paintpen.setWidth( TREE_WIDTH );
    if(m_finished_planning) {
      tree_painter.setOpacity( TREE_OPACITY );
    }
    tree_painter.setPen(tree_paintpen);
    if( m_string_class_index < 0 ) {
      for( list<MLRRTNode*>::iterator it= mp_tree->get_nodes().begin(); 
           it!=mp_tree->get_nodes().end();it++ ) {
        MLRRTNode* p_node = (*it);
        if(p_node) {
          if(p_node->mp_parent) {
            tree_painter.drawLine( toQPoint( p_node->mp_parent->m_pos ), 
                                   toQPoint( p_node->m_pos ) );
          }
        }
      }
    }
    else {
      StringClass* p_str_cls = mp_tree->get_expanding_tree_mgr()->get_string_classes()[ m_string_class_index ];
      if( p_str_cls ) {
        for( vector<ExpandingNode*>::iterator it_exp = p_str_cls->mp_exp_nodes.begin();
             it_exp != p_str_cls->mp_exp_nodes.end(); it_exp++ ) {
          ExpandingNode* p_exp_node = (*it_exp);
          if ( p_exp_node ) {
            for( list<MLRRTNode*>::iterator it = p_exp_node->mp_nodes.begin();
                 it != p_exp_node->mp_nodes.end(); it++ ) {
              MLRRTNode* p_node = (*it);
              if(p_node) {
                if(p_node->mp_parent) {
                  tree_painter.drawLine( toQPoint( p_node->mp_parent->m_pos ),
                                         toQPoint( p_node->m_pos ) );
                }
              }
            }
          }
        }
      }
    }
    tree_painter.end();
  }

  /* DRAW PATHS */ 
  if( m_show_paths ) {
    QPainter fpt_painter(device);
    QPen fpt_paintpen( PATH_COLOR );
    fpt_paintpen.setWidth( PATH_WIDTH );
    fpt_painter.setPen(fpt_paintpen);

    if( m_string_class_index < 0 ) {
      if(m_PPInfo.mp_found_paths.size() > 0 && m_found_path_index >= 0  ) {
        Path* p = m_PPInfo.mp_found_paths[m_found_path_index];
        int point_num = p->m_way_points.size();
        if(point_num > 0) {
          for(int i=0;i<point_num-1;i++) {
            fpt_painter.drawLine( toQPoint(p->m_way_points[i]), 
                                  toQPoint(p->m_way_points[i+1]) );
          }
        }
      }
    }
    else {
      StringClass* p_str_cls = mp_tree->get_expanding_tree_mgr()->get_string_classes()[ m_string_class_index ];
      if( p_str_cls ) {
        if ( p_str_cls->mp_path ) {
          int point_num = p_str_cls->mp_path->m_way_points.size();
          if(point_num > 0) {
            for(int i=0;i<point_num-1;i++) {
              fpt_painter.drawLine( toQPoint(p_str_cls->mp_path->m_way_points[i]),
                                    toQPoint(p_str_cls->mp_path->m_way_points[i+1]) );
            }
          }
        }
      }
    }
    fpt_painter.end();
  }

  /* DRAW DRAWED POINTS */
  if( m_show_points ) {
    if( m_drawed_points.size() > 1 ) {
      QPainter dp_painter(device);
      QPen dp_paintpen( DRAWING_LINE_COLOR );
      dp_paintpen.setWidth( LINE_WIDTH );
      dp_painter.setPen(dp_paintpen);
      for( unsigned int i = 0; i < m_drawed_points.size()-1; i++ ) {
        dp_painter.drawLine( m_drawed_points[ i ] ,
                             m_drawed_points[ i+1 ] );
      }
    }
  }

  /* DRAW REFERENCE FRAMES */
  if( m_show_reference_frames ) {
    QPainter rf_painter(device);
    QPen rf_paintpen( REFERENCE_FRAME_COLOR );
    rf_paintpen.setWidth( LINE_WIDTH );
    rf_painter.setPen(rf_paintpen);

    if ( m_reference_frame_index < 0 ) {
     for( unsigned int rf_i = 0; rf_i < m_viz_reference_frames.size(); rf_i ++ ) {
       ReferenceFrame* rf = m_viz_reference_frames[rf_i];
       rf_painter.drawLine( toQPoint( rf->m_segment.source() ),
                            toQPoint( rf->m_segment.target() ) );
      }
    }
    else{
      ReferenceFrame* rf = m_viz_reference_frames[m_reference_frame_index];
      rf_painter.drawLine( toQPoint( rf->m_segment.source() ),
                           toQPoint( rf->m_segment.target() ) );
    }
    rf_painter.end();
  }

  /* DRAW START AND GOAL */
  if(m_PPInfo.m_start.x() >= 0 && m_PPInfo.m_start.y() >= 0) {
    QPainter st_painter(device);
    QPen st_paintpen( START_COLOR );
    st_paintpen.setWidth( POINT_WIDTH );
    st_painter.setPen(st_paintpen);
    st_painter.drawPoint( m_PPInfo.m_start );
    st_painter.end();
  }

  if(m_PPInfo.m_goal.x() >= 0 && m_PPInfo.m_goal.y() >= 0) {
    QPainter gt_painter(device);
    QPen gt_paintpen( GOAL_COLOR );
    gt_paintpen.setWidth( POINT_WIDTH );
    gt_painter.setPen(gt_paintpen);
    gt_painter.drawPoint( m_PPInfo.m_goal );
    gt_painter.end();
  }

  if( NORMAL == m_mode ) {
    if ( m_item_selected_name != "" ) {
      QPainter text_painter(device);
      QPen text_pen( TEXT_COLOR );
      text_painter.setPen(text_pen);
      QRect rect = QRect( 5, 5, 100, 10 );
      text_painter.drawText( rect, Qt::AlignCenter, m_item_selected_name );
    }
  }
}

void MLRRTstarViz::paintEvent( QPaintEvent* e ) {
  QLabel::paintEvent(e);
  paint( this );
}

void MLRRTstarViz::set_show_reference_frames(bool show) {
  m_show_reference_frames = show;
  m_reference_frame_index = -1;
}

void MLRRTstarViz::set_show_subregions(bool show) {
  m_show_subregions = show;
  m_subregion_index = -1;
}

void MLRRTstarViz::set_show_paths(bool show) {
  m_show_paths = show;
  m_found_path_index = -1;
}

bool MLRRTstarViz::draw_path(QString filename) {

  QPixmap pix(m_PPInfo.m_objective_file);
  cout << "DUMP PATH IMG " << pix.width() << " " << pix.height() << endl;

  QFile file(filename);
  if(file.open(QIODevice::WriteOnly)) {
    if(m_PPInfo.mp_found_paths[ m_found_path_index ]) {
      draw_path_on_map(pix);
    }
    pix.save(&file, "PNG");
    return true;
  }
  return false;
}

bool MLRRTstarViz::save_current_viz(QString filename) {
  QPixmap pix(m_PPInfo.m_map_fullpath);
  QFile file(filename);
  if(file.open(QIODevice::WriteOnly)) {
    paint( dynamic_cast<QPaintDevice*>(&pix) );
    pix.save(&file, "PNG");
    return true;
  }
  return false;
}

void MLRRTstarViz::next_string_class() {
  if ( mp_tree ) {
    if ( mp_tree->get_expanding_tree_mgr() ) {
      ExpandingTreeMgr* p_mgr = mp_tree->get_expanding_tree_mgr();
      if ( m_string_class_index < p_mgr->get_string_classes().size()-1 ) {
        m_string_class_index ++;
        updateVizSubregions();
        updateVizReferenceFrames();
        m_found_path_index = -1;
      }
      else {
        m_string_class_index = -1;
        updateVizSubregions();
        updateVizReferenceFrames();
        m_found_path_index = -1;
      }
    }
  }
}

void MLRRTstarViz::prev_string_class() {
  if ( mp_tree ) {
    if ( mp_tree->get_expanding_tree_mgr() ) {
      ExpandingTreeMgr* p_mgr = mp_tree->get_expanding_tree_mgr();
      if ( m_string_class_index >= 0 ) {
        m_string_class_index --;
        updateVizSubregions();
        updateVizReferenceFrames();
        m_found_path_index = -1;
      }
      else {
        m_string_class_index = p_mgr->get_string_classes().size()-1;
        updateVizSubregions();
        updateVizReferenceFrames();
        m_found_path_index = -1;
      }
    }
  }
}

void MLRRTstarViz::prev_reference_frame() {
  if (m_show_reference_frames) {
    if ( m_reference_frame_index < 0) {
      m_reference_frame_index = m_viz_reference_frames.size()-1;
    }else{
      m_reference_frame_index -- ;
    }
  }
}

void MLRRTstarViz::next_reference_frame() {
  if (m_show_reference_frames) {
    if ( m_reference_frame_index >= m_viz_reference_frames.size()-1 ) {
      m_reference_frame_index = -1;
    }else{
      m_reference_frame_index ++;
    }
  }
}

string MLRRTstarViz::get_reference_frame_name() {

  if ( m_reference_frame_index < mp_reference_frames->get_reference_frames().size() ) {
    return mp_reference_frames->get_reference_frames()[m_reference_frame_index]->m_name;
  }
  return "NO REF FRAME";
}

string MLRRTstarViz::get_subregion_name() {
  SubRegion* p_subregion = get_selected_subregion();
  if( p_subregion ) {
    return p_subregion->get_name();
  }
  return "NO REGION";
}

void MLRRTstarViz::prev_found_path() {
  if ( m_PPInfo.mp_found_paths.size() == 0 ) {
    return;
  }
  if ( m_found_path_index < 0 ) {
    m_found_path_index = m_PPInfo.mp_found_paths.size() - 1;
  } else {
    m_found_path_index --;
  }
}

void MLRRTstarViz::next_found_path() {
  if ( m_PPInfo.mp_found_paths.size() == 0 ) {
    return;
  }
  if ( m_found_path_index >= m_PPInfo.mp_found_paths.size()-1 ) {
    m_found_path_index = -1;
  } else {
    m_found_path_index ++;
  }
}

void MLRRTstarViz::import_string_constraint( vector< QPoint > points, grammar_type_t type ) {
  vector< Point2D > ref_points;
  for( unsigned int i = 0; i < points.size(); i ++ ) {
    ref_points.push_back( Point2D( points[i].x(), points[i].y() ) );
  }
  if( mp_reference_frames ) {
    mp_reference_frames->import_string_constraint( ref_points, type );
  }
}

void MLRRTstarViz::mousePressEvent( QMouseEvent * event ) {
  // std::cout << "mousePressEvent" << std::endl;
  if ( event->button() == Qt::LeftButton ) {
    if( DRAWING == m_mode ) {
      m_dragging = true;
      m_show_points = true;
      m_drawed_points.clear();
    } 
    else if( NORMAL == m_mode ) {
      QPoint new_point( event->x(), event->y() );
      m_item_selected_name = item_selected( new_point );
      cout << "POS (" << event->x() << ", " << event->y() << ") " << m_item_selected_name.toStdString() << endl;
    }
  }
}

void MLRRTstarViz::mouseMoveEvent( QMouseEvent * event ) {
  // std::cout << "mouseMoveEvent " << mPoints.size() << std::endl;
  if( DRAWING == m_mode ) {
    if ( m_dragging == true ) {
      //std::cout << event->x() << " " << event->y() << std::endl;
      QPoint new_point( event->x(), event->y() );
      if( m_drawed_points.size() > 0 ) {
        QPoint last_point = m_drawed_points.back();
        if( abs( new_point.x() - last_point.x() ) > 1 &&
            abs( new_point.y() - last_point.y() ) > 1 ) {
          m_drawed_points.push_back( new_point );
        }
      }
      else {
        m_drawed_points.push_back( new_point );
      }
      repaint();
    }
  }
}

void MLRRTstarViz::mouseReleaseEvent( QMouseEvent * event ){
  // std::cout << "mouseReleaseEvent" << std::endl;
  if ( event->button() == Qt::LeftButton ) {
    if( DRAWING == m_mode ) {
      m_dragging = false;
    }
    else if( NORMAL == m_mode ) {
      m_item_selected_name = "";
    }
  }
}

ReferenceFrame* MLRRTstarViz::get_selected_reference_frame() {

  if ( m_reference_frame_index >= mp_reference_frames->get_reference_frames().size() ) {
    return NULL;
  }
  if ( m_reference_frame_index < 0 ) {
    return NULL;
  }
  return mp_reference_frames->get_reference_frames()[ m_reference_frame_index ];
}
    
SubRegion* MLRRTstarViz::get_selected_subregion() {
  if( m_subregion_index >= 0 && m_subregion_index < m_viz_subregions.size() ) {
    return m_viz_subregions[ m_subregion_index ];
  } 
  return NULL;
}

 
void MLRRTstarViz::prev_subregion() {
  if (m_show_subregions) {
    if( m_subregion_index > 0 ) {
      m_subregion_index --;
    }
    else {
      m_subregion_index = m_viz_subregions.size()-1;
    }
  }
}

void MLRRTstarViz::next_subregion() {
  if (m_show_subregions) {
    if( m_subregion_index < m_viz_subregions.size()-1 ) {
      m_subregion_index ++;
    }
    else {
      m_subregion_index = 0;
    } 
  }
}

void MLRRTstarViz::draw_path_on_map(QPixmap& map) {

  Path * p = m_PPInfo.mp_found_paths[ m_found_path_index ];
  QPainter painter(&map);
  QPen paintpen(QColor(255,140,0));
  paintpen.setWidth(2);
  painter.setPen(paintpen);

  int point_num = p->m_way_points.size();

  if(point_num > 0) {
    for(int i=0;i<point_num-1;i++) {
      painter.drawLine( toQPoint( p->m_way_points[i] ), toQPoint( p->m_way_points[i+1] ) );
    }
  }
  painter.end();

  QPainter startPainter(&map);
  QPen paintpen1(QColor(255,0,0));
  paintpen.setWidth(10);
  startPainter.setPen(paintpen1);
  startPainter.end();

  startPainter.drawPoint( toQPoint( p->m_way_points[0] ) );
  int lastIdx = p->m_way_points.size() - 1;
  QPainter endPainter(&map);
  QPen paintpen2(QColor(0,0,255));
  paintpen.setWidth(10);
  endPainter.setPen(paintpen2);
  endPainter.drawPoint( toQPoint( p->m_way_points[lastIdx] ) );
  endPainter.end();
}

QString MLRRTstarViz::get_string_class_info() {
  QString info;
  if( mp_tree ) {
    if( mp_tree->get_expanding_tree_mgr() ) {
      if( m_string_class_index < 0 ) {
        info = "ALL";
      }
      else {
        info = QString::fromStdString( mp_tree->get_expanding_tree_mgr()->get_string_classes()[ m_string_class_index ]->get_name() ); 
      }
    }
  }
  return info;
}

QString MLRRTstarViz::item_selected( QPoint pos ) {
  QString name = "";
  Point2D point = toPoint2D( pos );

  if ( mp_reference_frames ) {
    LineSubSegment* p_line_sub_segment = mp_reference_frames->get_world_map()->find_linesubsegment( point );
    if( p_line_sub_segment ) {
      cout << "FIND " << p_line_sub_segment->get_name() << endl;
      return QString::fromStdString( p_line_sub_segment->get_name() );
    }
    SubRegion* p_subregion = mp_reference_frames->get_world_map()->find_subregion( point );
    if( p_subregion ) {
      cout << "FIND " << p_subregion->get_name() << endl;
      return QString::fromStdString( p_subregion->get_name() );
    }
  }
  return name;
}
