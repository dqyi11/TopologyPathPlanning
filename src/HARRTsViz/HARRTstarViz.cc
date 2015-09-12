#include <QtGui>

#include "HARRTstarViz.h"

#define START_TREE_COLOR      QColor(160,160,0)
#define GOAL_TREE_COLOR       QColor(0,160,160)
#define START_COLOR           QColor(255,0,0)
#define GOAL_COLOR            QColor(0,0,255)
#define REFERENCE_FRAME_COLOR QColor(0,255,0)
#define PATH_COLOR            QColor(255,153,21)

HARRTstarViz::HARRTstarViz( QWidget *parent ) :
    QLabel(parent) {
    mp_tree = NULL;
    m_show_reference_frames = false;
    m_show_regions = false;
    m_reference_frame_index = -1;
    m_found_path_index = -1;
    mp_reference_frames = NULL;
    m_colors.clear();
}

void HARRTstarViz::setTree( HARRTstar* p_tree ) {
    mp_tree = p_tree;
}

void HARRTstarViz::setReferenceFrameSet(ReferenceFrameSet* p_rf) {
    mp_reference_frames = p_rf;
    for( unsigned int i = 0; i < mp_reference_frames->get_world_map()->get_subregion_set().size(); i++) {
        m_colors.push_back( QColor( rand()%255, rand()%255, rand()%255 ) );
    }
}

void HARRTstarViz::paintEvent( QPaintEvent * e ) {
    QLabel::paintEvent(e);

    if(m_show_regions) {
        for( unsigned int i = 0; i < mp_reference_frames->get_world_map()->get_subregion_set().size(); i++) {
            SubRegionSet* p_subregion_set = mp_reference_frames->get_world_map()->get_subregion_set()[i];
            if(p_subregion_set) {
                QPainter rg_painter(this);
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
            }
        }
    }

    if(mp_tree) {

        QPainter st_tree_painter(this);
        QPen st_tree_paintpen(START_TREE_COLOR);
        st_tree_paintpen.setWidth(1);
        st_tree_painter.setPen(st_tree_paintpen);
        for( std::list<RRTNode*>::iterator it= mp_tree->get_st_nodes().begin(); it!=mp_tree->get_st_nodes().end();it++ ) {
            RRTNode* p_node = (*it);
            if(p_node) {
                if(p_node->mp_parent) {
                    st_tree_painter.drawLine(QPoint(p_node->m_pos[0], p_node->m_pos[1]), QPoint(p_node->mp_parent->m_pos[0], p_node->mp_parent->m_pos[1]));
                }
            }
        }
        QPainter gt_tree_painter(this);
        QPen gt_tree_paintpen(GOAL_TREE_COLOR);
        gt_tree_paintpen.setWidth(1);
        gt_tree_painter.setPen(gt_tree_paintpen);
        for( std::list<RRTNode*>::iterator it= mp_tree->get_gt_nodes().begin(); it!=mp_tree->get_gt_nodes().end();it++ ) {
            RRTNode* p_node = (*it);
            if(p_node) {
                if(p_node->mp_parent) {
                    gt_tree_painter.drawLine(QPoint(p_node->m_pos[0], p_node->m_pos[1]), QPoint(p_node->mp_parent->m_pos[0], p_node->mp_parent->m_pos[1]));
                }
            }
        }
        
        if(m_PPInfo.mp_found_paths.size() > 0 && m_found_path_index >= 0  ) {
            Path * p = m_PPInfo.mp_found_paths[m_found_path_index];
            QPainter painter(this);
            QPen paintpen(QColor(255,140,0));
            paintpen.setWidth(2);
            painter.setPen(paintpen);

            int point_num = p->m_way_points.size();

            if(point_num > 0) {
                for(int i=0;i<point_num-1;i++) {
                    painter.drawLine(QPoint(p->m_way_points[i][0], p->m_way_points[i][1]), QPoint(p->m_way_points[i+1][0], p->m_way_points[i+1][1]));
                }
            }
        }
    }

    if(m_PPInfo.m_start.x() >= 0 && m_PPInfo.m_start.y() >= 0) {
        QPainter st_painter(this);
        QPen st_paintpen( START_COLOR );
        st_paintpen.setWidth(8);
        st_painter.setPen(st_paintpen);
        st_painter.drawPoint(m_PPInfo.m_start);
    }

    if(m_PPInfo.m_goal.x() >= 0 && m_PPInfo.m_goal.y() >= 0) {
        QPainter gt_painter(this);
        QPen gt_paintpen( GOAL_COLOR );
        gt_paintpen.setWidth(8);
        gt_painter.setPen(gt_paintpen);
        gt_painter.drawPoint(m_PPInfo.m_goal);
    }

    if( m_show_reference_frames ) {
        if( mp_reference_frames ) {
            QPainter rf_painter(this);
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
        }
    }

    if( mp_tree ) {
        if( mp_tree->get_string_class_mgr() ) {
            std::vector< StringClass* > classes = mp_tree->get_string_class_mgr()->get_string_classes();
            QPainter path_painter(this);
            QPen path_paintpen( PATH_COLOR );
            path_paintpen.setWidth(2);
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
        }
    }
}

void HARRTstarViz::set_show_reference_frames(bool show) {
    m_show_reference_frames = show;
    m_reference_frame_index = 0;
}

void HARRTstarViz::set_show_regions(bool show) {
    m_show_regions = show;
}

bool HARRTstarViz::drawPath(QString filename) {

    QPixmap pix(m_PPInfo.m_objective_file);

    std::cout << "DUMP PATH IMG " << pix.width() << " " << pix.height() << std::endl;

    QFile file(filename);
    if(file.open(QIODevice::WriteOnly)) {
        if(m_PPInfo.mp_found_paths[ m_found_path_index ]) {
            drawPathOnMap(pix);
        }
        pix.save(&file, "PNG");
        return true;
    }
    return false;
}

void HARRTstarViz::drawPathOnMap(QPixmap& map) {

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

void HARRTstarViz::prev_reference_frame() {
    if (m_show_reference_frames) {
        if ( m_reference_frame_index <= 0) {
            m_reference_frame_index = mp_reference_frames->get_reference_frames().size();
        }else{
            m_reference_frame_index -- ;
        }
    }
}

void HARRTstarViz::next_reference_frame() {
    if (m_show_reference_frames) {
        if ( m_reference_frame_index >= mp_reference_frames->get_reference_frames().size() ) {
            m_reference_frame_index = 0;
        }else{
            m_reference_frame_index ++;
        }
    }
}

std::string HARRTstarViz::get_reference_frame_name() {

    if ( m_reference_frame_index < mp_reference_frames->get_reference_frames().size() ) {
        return mp_reference_frames->get_reference_frames()[m_reference_frame_index]->m_name;
    }
    return "";
}

void HARRTstarViz::prev_found_path() {
    if ( m_PPInfo.mp_found_paths.size() == 0 ) {
        return;
    }
    if ( m_found_path_index < 0 ) {
        m_found_path_index = m_PPInfo.mp_found_paths.size() - 1;
    } else {
        m_found_path_index --;
    }
}

void HARRTstarViz::next_found_path() {
    if ( m_PPInfo.mp_found_paths.size() == 0 ) {
        return;
    }
    if ( m_found_path_index >= m_PPInfo.mp_found_paths.size()-1 ) {
        m_found_path_index = -1;
    } else {
        m_found_path_index ++;
    }
}
