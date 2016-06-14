#include <QtGui>

#include "birrtstar_viz.h"

#define START_TREE_COLOR        QColor(160,160,0)
#define START_TREE_COLOR_ALPHA  QColor(160,160,0,100)
#define GOAL_TREE_COLOR         QColor(0,160,160)
#define GOAL_TREE_COLOR_ALPHA   QColor(0,160,160,100)
#define START_COLOR             QColor(255,0,0)
#define GOAL_COLOR              QColor(0,0,255)
#define REFERENCE_FRAME_COLOR   QColor(0,255,0)
#define PATH_COLOR              QColor(255,153,21)
#define DRAWING_LINE_COLOR      QColor(153,76,0)
#define LINE_WIDTH              2

using namespace homotopy;
using namespace birrts;

BIRRTstarViz::BIRRTstarViz( QWidget *parent ) :
    QLabel(parent) {
  mp_tree = NULL;
  m_show_reference_frames = false;
  m_show_regions = false;
  m_finished_planning = false;
  m_reference_frame_index = -1;
  m_found_path_index = -1;
  m_region_index = -1;
  m_subregion_index = -1;
  mp_reference_frames = NULL;
  m_tree_show_type = BOTH_TREES_SHOW;
  m_show_points = false;
  m_colors.clear();
}

void BIRRTstarViz::set_tree( BIRRTstar* p_tree ) {
  mp_tree = p_tree;
}

void BIRRTstarViz::set_reference_frame_set(ReferenceFrameSet* p_rf) {
  mp_reference_frames = p_rf;
  for( unsigned int i = 0; i < mp_reference_frames->get_world_map()->get_subregion_set().size(); i++) {
    m_colors.push_back( QColor( rand()%255, rand()%255, rand()%255 ) );
  }
}

void BIRRTstarViz::paintEvent( QPaintEvent * e ) {
  QLabel::paintEvent(e);

  paint(this);
}


void BIRRTstarViz::paint(QPaintDevice * device) {
  if(mp_reference_frames==NULL) {
    return;
  }
  if(mp_reference_frames->get_world_map()==NULL) {
    return;
  }

  if(m_show_regions) {

    if( m_region_index < 0 ) {
      for( unsigned int i = 0; i < mp_reference_frames->get_world_map()->get_subregion_set().size(); i++) {
        SubRegionSet* p_subregion_set = mp_reference_frames->get_world_map()->get_subregion_set()[i];
        if(p_subregion_set) {
          QPainter rg_painter(device);
          rg_painter.setRenderHint(QPainter::Antialiasing);
          QBrush rg_brush( m_colors[i] );
          rg_painter.setPen(Qt::NoPen);
          for( unsigned int j = 0; j < p_subregion_set->m_subregions.size(); j ++ ) {
            SubRegion* p_subreg = p_subregion_set->m_subregions[j];
            if(p_subreg) {
              QPolygon poly;
              for( unsigned int k = 0; k < p_subreg->m_points.size(); k++) {
                double x = CGAL::to_double( p_subreg->m_points[k].x() );
                double y = CGAL::to_double( p_subreg->m_points[k].y() );
                poly << QPoint(x , y);
              }
              QPainterPath tmpPath;
              tmpPath.addPolygon(poly);
              rg_painter.fillPath(tmpPath, rg_brush);
            }
          }
          rg_painter.end();
        }
      }
    }
    else {
      SubRegionSet* p_subregion_set = mp_reference_frames->get_world_map()->get_subregion_set()[ m_region_index ];
      if(p_subregion_set) {
        QPainter rg_painter(device);
        rg_painter.setRenderHint(QPainter::Antialiasing);
        QBrush rg_brush( m_colors[m_region_index] );
        rg_painter.setPen(Qt::NoPen);

        if( m_subregion_index < 0 ) {
          for( unsigned int j = 0; j < p_subregion_set->m_subregions.size(); j ++ ) {
            SubRegion* p_subreg = p_subregion_set->m_subregions[j];
            if(p_subreg) {
              QPolygon poly;
              for( unsigned int k = 0; k < p_subreg->m_points.size(); k++) {
                double x = CGAL::to_double( p_subreg->m_points[k].x() );
                double y = CGAL::to_double( p_subreg->m_points[k].y() );
                poly << QPoint(x , y);
              }
              QPainterPath tmpPath;
              tmpPath.addPolygon(poly);
              rg_painter.fillPath(tmpPath, rg_brush);
            }
          }
        }
        else {
          SubRegion* p_subreg = p_subregion_set->m_subregions[m_subregion_index];
          if(p_subreg) {
            QPolygon poly;
            for( unsigned int k = 0; k < p_subreg->m_points.size(); k++) {
              double x = CGAL::to_double( p_subreg->m_points[k].x() );
              double y = CGAL::to_double( p_subreg->m_points[k].y() );
              poly << QPoint(x , y);
            }
            QPainterPath tmpPath;
            tmpPath.addPolygon(poly);
            rg_painter.fillPath(tmpPath, rg_brush);
          }
        }
        rg_painter.end();
      } 
    }
  }

  if(mp_tree) {
    if(m_tree_show_type == START_TREE_SHOW || m_tree_show_type == BOTH_TREES_SHOW ) {
      QPainter st_tree_painter(device);
      QPen st_tree_paintpen;
      if(m_finished_planning) {
        st_tree_paintpen.setColor(START_TREE_COLOR_ALPHA);
      }
      else{
        st_tree_paintpen.setColor(START_TREE_COLOR);
      }
      st_tree_paintpen.setWidth(1);
      if(m_finished_planning) {
        st_tree_painter.setOpacity(0.4);
      }
      st_tree_painter.setPen(st_tree_paintpen);
      for( std::list<BIRRTNode*>::iterator it= mp_tree->get_st_nodes().begin(); it!=mp_tree->get_st_nodes().end();it++ ) {
        BIRRTNode* p_node = (*it);
        if(p_node) {
          if(p_node->mp_parent) {
            st_tree_painter.drawLine(QPoint(p_node->m_pos[0], p_node->m_pos[1]), QPoint(p_node->mp_parent->m_pos[0], p_node->mp_parent->m_pos[1]));
          }
        }
      }
      st_tree_painter.end();
    }
    if(m_tree_show_type == GOAL_TREE_SHOW || m_tree_show_type == BOTH_TREES_SHOW) {
      QPainter gt_tree_painter(device);
      QPen gt_tree_paintpen;
      if(m_finished_planning) {
        gt_tree_paintpen.setColor(GOAL_TREE_COLOR_ALPHA);
      }
      else{
        gt_tree_paintpen.setColor(GOAL_TREE_COLOR);
      }
      gt_tree_paintpen.setWidth(1);
      if(m_finished_planning) {
        gt_tree_painter.setOpacity(0.4);
      }
      gt_tree_painter.setPen(gt_tree_paintpen);
      for( std::list<BIRRTNode*>::iterator it= mp_tree->get_gt_nodes().begin(); it!=mp_tree->get_gt_nodes().end();it++ ) {
        BIRRTNode* p_node = (*it);
        if(p_node) {
          if(p_node->mp_parent) {
            gt_tree_painter.drawLine(QPoint(p_node->m_pos[0], p_node->m_pos[1]), QPoint(p_node->mp_parent->m_pos[0], p_node->mp_parent->m_pos[1]));
          }
        }
      }
      gt_tree_painter.end();
    }
  } 
  if(m_PPInfo.mp_found_paths.size() > 0 && m_found_path_index >= 0  ) {
    Path* p = m_PPInfo.mp_found_paths[m_found_path_index];
    QPainter fpt_painter(device);
    QPen fpt_paintpen(QColor(255,140,0));
    fpt_paintpen.setWidth(4);
    fpt_painter.setPen(fpt_paintpen);

    int point_num = p->m_way_points.size();
    if(point_num > 0) {
      for(int i=0;i<point_num-1;i++) {
        fpt_painter.drawLine(QPoint(p->m_way_points[i][0], p->m_way_points[i][1]), QPoint(p->m_way_points[i+1][0], p->m_way_points[i+1][1]));
      }
    }
    fpt_painter.end();
  }
    
  if(m_PPInfo.m_start.x() >= 0 && m_PPInfo.m_start.y() >= 0) {
    QPainter st_painter(device);
    QPen st_paintpen( START_COLOR );
    st_paintpen.setWidth(8);
    st_painter.setPen(st_paintpen);
    st_painter.drawPoint(m_PPInfo.m_start);
    st_painter.end();
  }

  if(m_PPInfo.m_goal.x() >= 0 && m_PPInfo.m_goal.y() >= 0) {
    QPainter gt_painter(device);
    QPen gt_paintpen( GOAL_COLOR );
    gt_paintpen.setWidth(8);
    gt_painter.setPen(gt_paintpen);
    gt_painter.drawPoint(m_PPInfo.m_goal);
    gt_painter.end();
  }

  if( m_show_reference_frames ) {
    if( mp_reference_frames ) {
      QPainter rf_painter(device);
      QPen rf_paintpen( REFERENCE_FRAME_COLOR );
      rf_paintpen.setWidth(2);
      rf_painter.setPen(rf_paintpen);

      if ( m_reference_frame_index >= mp_reference_frames->get_reference_frames().size() ) {
        for( unsigned int rf_i = 0; rf_i < mp_reference_frames->get_reference_frames().size(); rf_i ++ ) {
          ReferenceFrame* rf = mp_reference_frames->get_reference_frames()[rf_i];
          rf_painter.drawLine( QPoint( CGAL::to_double(rf->m_segment.source().x()), CGAL::to_double(rf->m_segment.source().y()) ),
                               QPoint( CGAL::to_double(rf->m_segment.target().x()), CGAL::to_double(rf->m_segment.target().y()) ) );
        }
      }
      else{
        ReferenceFrame* rf = mp_reference_frames->get_reference_frames()[m_reference_frame_index];
        rf_painter.drawLine( QPoint( CGAL::to_double(rf->m_segment.source().x()), CGAL::to_double(rf->m_segment.source().y()) ),
                             QPoint( CGAL::to_double(rf->m_segment.target().x()), CGAL::to_double(rf->m_segment.target().y()) ) );
      }
      rf_painter.end();
    }
  }

  if( mp_tree!=NULL && m_finished_planning==false ) {
    if( mp_tree->get_string_class_mgr() ) {
      std::vector< StringClass* > classes = mp_tree->get_string_class_mgr()->get_string_classes();
      QPainter path_painter(device);
      QPen path_paintpen( PATH_COLOR );
      path_paintpen.setWidth(3);
      path_paintpen.setStyle( Qt::DashDotLine );
      path_painter.setPen(path_paintpen);
      for( unsigned int i = 0; i < classes.size(); i ++ ) {
        Path* p_path = classes[i]->mp_path;
        if( p_path ) {
          if( p_path->m_way_points.size() > 0 ) {
            for( unsigned int j = 0; j < p_path->m_way_points.size()-1; j ++ ){
              path_painter.drawLine( QPoint( p_path->m_way_points[j][0],
                                             p_path->m_way_points[j][1] ), 
                                     QPoint( p_path->m_way_points[j+1][0],
                                             p_path->m_way_points[j+1][1] ) );
            }
          }
        } 
      }
      path_painter.end();
    }
  }

  if( m_show_points ) {
    QPainter draw_line_painter(device);
    QPen draw_line_pen( DRAWING_LINE_COLOR );
    draw_line_pen.setWidth( LINE_WIDTH );
    draw_line_painter.setPen(draw_line_pen);
    if( m_drawed_points.size() > 1 ) {
      for( unsigned int pi = 0; pi < m_drawed_points.size()-1 ; pi ++ ) {
        draw_line_painter.drawLine( m_drawed_points[pi], m_drawed_points[pi+1] );
      }
    }
    draw_line_painter.end();
  }
}

void BIRRTstarViz::set_show_reference_frames(bool show) {
  m_show_reference_frames = show;
  m_reference_frame_index = 0;
}

void BIRRTstarViz::set_show_regions(bool show) {
  m_show_regions = show;
}

bool BIRRTstarViz::draw_path(QString filename) {

  QPixmap pix(m_PPInfo.m_objective_file);

  std::cout << "DUMP PATH IMG " << pix.width() << " " << pix.height() << std::endl;

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

bool BIRRTstarViz::save_current_viz(QString filename) {
  QPixmap pix(m_PPInfo.m_map_fullpath);
  QFile file(filename);
  if(file.open(QIODevice::WriteOnly)) {
    paint( dynamic_cast<QPaintDevice*>(&pix) );
    pix.save(&file, "PNG");
    return true;
  }
  return false;
}

void BIRRTstarViz::draw_path_on_map(QPixmap& map) {

  Path * p = m_PPInfo.mp_found_paths[ m_found_path_index ];
  QPainter painter(&map);
  QPen paintpen(QColor(255,140,0));
  paintpen.setWidth(2);
  painter.setPen(paintpen);

  int point_num = p->m_way_points.size();

  if(point_num > 0) {
    for(int i=0;i<point_num-1;i++) {
      painter.drawLine( QPoint(p->m_way_points[i][0], p->m_way_points[i][1]), QPoint(p->m_way_points[i+1][0], p->m_way_points[i+1][1]) );
    }
  }

  painter.end();

  QPainter startPainter(&map);
  QPen paintpen1(QColor(255,0,0));
  paintpen.setWidth(10);
  startPainter.setPen(paintpen1);
  startPainter.end();

  startPainter.drawPoint( QPoint(p->m_way_points[0][0], p->m_way_points[0][1]) );
  int lastIdx = p->m_way_points.size() - 1;
  QPainter endPainter(&map);
  QPen paintpen2(QColor(0,0,255));
  paintpen.setWidth(10);
  endPainter.setPen(paintpen2);
  endPainter.drawPoint( QPoint(p->m_way_points[lastIdx][0], p->m_way_points[lastIdx][1]) );
  endPainter.end();
        
}

void BIRRTstarViz::switch_tree_show_type() {

  switch(m_tree_show_type) {
  case NONE_TREE_SHOW:
    m_tree_show_type = START_TREE_SHOW;
    break;
  case START_TREE_SHOW:
    m_tree_show_type = GOAL_TREE_SHOW;
    break;
  case GOAL_TREE_SHOW:
    m_tree_show_type = BOTH_TREES_SHOW;
    break;
  case BOTH_TREES_SHOW:
    m_tree_show_type = NONE_TREE_SHOW;
    break;
  }
}

void BIRRTstarViz::prev_reference_frame() {
  if(mp_reference_frames) {
    if (m_show_reference_frames) {
      if ( m_reference_frame_index <= 0) {
        m_reference_frame_index = mp_reference_frames->get_reference_frames().size();
      }else{
        m_reference_frame_index -- ;
      }
    }
  }
}

void BIRRTstarViz::next_reference_frame() {
  if(mp_reference_frames) {
    if (m_show_reference_frames) {
      if ( m_reference_frame_index >= mp_reference_frames->get_reference_frames().size() ) {
        m_reference_frame_index = 0;
      }else{
        m_reference_frame_index ++;
      }
    }
  }
}

std::string BIRRTstarViz::get_reference_frame_name() {

  if (mp_reference_frames) {
    if ( m_reference_frame_index < mp_reference_frames->get_reference_frames().size() ) {
      return mp_reference_frames->get_reference_frames()[m_reference_frame_index]->m_name;
    }
  }
  return "NO REF FRAME";
}

std::string BIRRTstarViz::get_region_name() {
  SubRegion* p_subregion = get_selected_subregion();
  if( p_subregion ) {
    return p_subregion->get_name();
  }
  SubRegionSet* p_subregion_set = get_selected_subregion_set();
  if( p_subregion_set ) {
    return p_subregion_set->get_name();
  }
  return "NO REGION";
}

void BIRRTstarViz::prev_found_path() {
  if ( m_PPInfo.mp_found_paths.size() == 0 ) {
    return;
  }
  if ( m_found_path_index < 0 ) {
    m_found_path_index = m_PPInfo.mp_found_paths.size() - 1;
  } else {
    m_found_path_index --;
  }
}

void BIRRTstarViz::next_found_path() {
  if ( m_PPInfo.mp_found_paths.size() == 0 ) {
    return;
  }
  if ( m_found_path_index >= m_PPInfo.mp_found_paths.size()-1 ) {
    m_found_path_index = -1;
  } else {
    m_found_path_index ++;
  }
}

void BIRRTstarViz::import_string_constraint( std::vector< QPoint > points, grammar_type_t type ) {
  std::vector< Point2D > ref_points;
  for( unsigned int i = 0; i < points.size(); i ++ ) {
    ref_points.push_back( Point2D( points[i].x(), points[i].y() ) );
  }
  if( mp_reference_frames ) {
    mp_reference_frames->import_string_constraint( ref_points, type );
  }
}

void BIRRTstarViz::mousePressEvent( QMouseEvent * event ) {
  // std::cout << "mousePressEvent" << std::endl;
  if ( event->button() == Qt::LeftButton ) {
    m_dragging = true;
    m_show_points = true;
    m_drawed_points.clear();
  }
}

void BIRRTstarViz::mouseMoveEvent( QMouseEvent * event ) {
  // std::cout << "mouseMoveEvent " << mPoints.size() << std::endl;
  if ( m_dragging == true ) {
    //std::cout << event->x() << " " << event->y() << std::endl;
    QPoint new_point( event->x(), event->y() );
    if( m_drawed_points.size() > 0 ) {
      QPoint last_point = m_drawed_points.back();
      if( std::abs( new_point.x() - last_point.x() ) > 1 &&
          std::abs( new_point.y() - last_point.y() ) > 1 ) {
        m_drawed_points.push_back( new_point );
      }
    }
    else {
      m_drawed_points.push_back( new_point );
    }
    repaint();
  }
}

void BIRRTstarViz::mouseReleaseEvent( QMouseEvent * event ){
  // std::cout << "mouseReleaseEvent" << std::endl;
  if ( event->button() == Qt::LeftButton ) {
    m_dragging = false;
  }
}

ReferenceFrame* BIRRTstarViz::get_selected_reference_frame() {

  if(mp_reference_frames == NULL) {
      return NULL;
  }
  if ( m_reference_frame_index >= mp_reference_frames->get_reference_frames().size() ) {
    return NULL;
  }
  if ( m_reference_frame_index < 0 ) {
    return NULL;
  }
  return mp_reference_frames->get_reference_frames()[ m_reference_frame_index ];
}
    
SubRegionSet* BIRRTstarViz::get_selected_subregion_set() {

  if ( mp_reference_frames == NULL ) {
      return NULL;
  }
  if ( m_region_index >= mp_reference_frames->get_world_map()->get_subregion_set().size() ) {
    return NULL;
  }
  if ( m_region_index < 0 ) {
    return NULL;
  }
  return mp_reference_frames->get_world_map()->get_subregion_set()[ m_region_index ];
}

SubRegion* BIRRTstarViz::get_selected_subregion() {

  SubRegionSet* p_subregion_set = get_selected_subregion_set();
  if (p_subregion_set) {
    if( m_subregion_index >= 0 && m_subregion_index < p_subregion_set->m_subregions.size() ) {
      return p_subregion_set->m_subregions[ m_subregion_index ];
    } 
    return NULL;
  } 
  return NULL;
}

void BIRRTstarViz::prev_region() {
  if(mp_reference_frames) {
    if (m_show_regions) {
      if ( m_region_index < 0) {
        m_region_index = mp_reference_frames->get_world_map()->get_subregion_set().size() - 1;
        m_subregion_index = -1;
      }else{
        m_region_index -- ;
        m_subregion_index = -1;
      }
    }
  }
}

void BIRRTstarViz::next_region() {
  if(mp_reference_frames) {
    if (m_show_regions) {
      if ( m_region_index >= mp_reference_frames->get_world_map()->get_subregion_set().size()-1 ) {
        m_region_index = -1;
        m_subregion_index = -1;
      }else{
        m_region_index ++;
        m_subregion_index = -1;
      }
    }
  }
}
 
void BIRRTstarViz::prev_subregion() {
  if(mp_reference_frames) {
    if (m_show_regions) {
      if( m_region_index >= 0 && m_region_index < mp_reference_frames->get_world_map()->get_subregion_set().size()-1 ) {
        SubRegionSet* p_subregions = mp_reference_frames->get_world_map()->get_subregion_set()[ m_region_index ];
        if( m_subregion_index > 0 ) {
          m_subregion_index --;
        }
        else {
          m_subregion_index = p_subregions->m_subregions.size()-1;
        }
      }
    }
  }
}

void BIRRTstarViz::next_subregion() {
  if(mp_reference_frames) {
    if (m_show_regions) {
      if( m_region_index >= 0 && m_region_index < mp_reference_frames->get_world_map()->get_subregion_set().size()-1 ) {
        SubRegionSet* p_subregions = mp_reference_frames->get_world_map()->get_subregion_set()[ m_region_index ];
        if( m_subregion_index < p_subregions->m_subregions.size()-1 ) {
          m_subregion_index ++;
        }
        else {
          m_subregion_index = 0;
        }
      }
    }
  }
}

void BIRRTstarViz::reset() {
  mp_tree = NULL;
  m_finished_planning = false;
  m_reference_frame_index = -1;
  m_found_path_index = -1;
  m_region_index = -1;
  m_subregion_index = -1;
  m_tree_show_type = BOTH_TREES_SHOW;
  m_show_points = false;
}
