#include <QtGui>

#include "rrtstar_viz.h"

RRTstarViz::RRTstarViz( QWidget *parent ) :
    QLabel(parent) {
    mp_tree = NULL;
}

void RRTstarViz::setTree( RRTstar* p_tree ) {
    mp_tree = p_tree;
}


void RRTstarViz::paintEvent( QPaintEvent * e ) {
    QLabel::paintEvent(e);

    if(mp_tree) {

        QPainter painter(this);
        QPen paintpen(QColor(0,255,0));
        paintpen.setWidth(2);
        painter.setPen(paintpen);

        for( std::list<RRTNode*>::iterator it= mp_tree->get_nodes().begin(); it!=mp_tree->get_nodes().end();it++ ) {
            RRTNode* p_node = (*it);
            if(p_node) {
                if(p_node->mp_parent) {
                    painter.drawLine(QPoint(p_node->m_pos[0], p_node->m_pos[1]), QPoint(p_node->mp_parent->m_pos[0], p_node->mp_parent->m_pos[1]));
                }

                /*
                for(std::list<RRTNode*>::iterator itc= p_node->m_child_nodes.begin(); itc!=p_node->m_child_nodes.end();itc++) {
                    RRTNode* p_child_node = (*itc);
                    if(p_child_node)
                    {
                        painter.drawLine(QPoint(p_node->m_pos[0], p_node->m_pos[1]), QPoint(p_child_node->m_pos[0], p_child_node->m_pos[1]));
                    }
                }*/
            }
        }

        if(m_PPInfo.mp_found_path) {
            Path * p = m_PPInfo.mp_found_path;
            QPainter painter(this);
            QPen paintpen(QColor(255,140,0));
            paintpen.setWidth(4);
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
        QPainter painter(this);
        QPen paintpen(QColor(255,0,0));
        paintpen.setWidth(10);
        painter.setPen(paintpen);
        painter.drawPoint(m_PPInfo.m_start);
    }

    if(m_PPInfo.m_goal.x() >= 0 && m_PPInfo.m_goal.y() >= 0) {
        QPainter painter(this);
        QPen paintpen(QColor(0,0,255));
        paintpen.setWidth(10);
        painter.setPen(paintpen);
        painter.drawPoint(m_PPInfo.m_goal);
    }
}

bool RRTstarViz::drawPath(QString filename) {

    QPixmap pix(m_PPInfo.m_objective_file);

    std::cout << "DUMP PATH IMG " << pix.width() << " " << pix.height() << std::endl;

    QFile file(filename);
    if(file.open(QIODevice::WriteOnly)) {
        if(m_PPInfo.mp_found_path) {
            drawPathOnMap(pix);
        }
        pix.save(&file, "PNG");
        return true;
    }
    return false;
}

void RRTstarViz::drawPathOnMap(QPixmap& map) {

    Path * p = m_PPInfo.mp_found_path;
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
    startPainter.drawPoint( QPoint(p->m_way_points[0][0], p->m_way_points[0][1]) );
    startPainter.end();

    int lastIdx = p->m_way_points.size() - 1;
    QPainter endPainter(&map);
    QPen paintpen2(QColor(0,0,255));
    paintpen.setWidth(10);
    endPainter.setPen(paintpen2);
    endPainter.drawPoint( QPoint(p->m_way_points[lastIdx][0], p->m_way_points[lastIdx][1]) );
    endPainter.end();


}
