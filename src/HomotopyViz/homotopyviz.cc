#include <cstdlib>
#include <QPainter>
#include <QMouseEvent>
#include "homotopyviz.h"
#include "img_load_util.h"

#define LINE_WIDTH             1
#define LINE_WIDTH_HIGHLIGHTED 5
#define POINT_SIZE             4
#define ALPHA_COLOR            QColor(0,0,255)
#define BETA_COLOR             QColor(0,255,0)
#define CENTER_POINT_COLOR     QColor(255,0,0)
#define BK_COLOR               QColor(255,140,0)
#define INTERSECTION_COLOR     QColor(160,160,160)
#define TEXT_COLOR             QColor(0,0,0)
#define OBSTACLE_COLOR         QColor(125,125,125)
#define LINE_HIGHLIGHTED_COLOR QColor(204,204,0)
#define DRAWING_LINE_COLOR     QColor(153,76,0)


HomotopyViz::HomotopyViz(QWidget *parent) :
    QLabel(parent) {

    mpWorld = NULL;
    mWorldWidth = 0;
    mWorldHeight = 0;
    mShowSubregion = false;
    mShowSubsegment = true;
    mRegionIdx = -1;
    mSubRegionIdx = 0;
    mDragging = false;
}

bool HomotopyViz::loadMap( QString filename ) {

    QPixmap pix(filename);
    if( pix.isNull() == true ) {
        return false;
    }
    QPixmap emptyPix(pix.width(), pix.height());
    emptyPix.fill(QColor("white"));
    std::cout << " EMPTY PIX " << emptyPix.width() << " * " << emptyPix.height() << std::endl;
    //setPixmap(pix);
    setPixmap(emptyPix);
    initWorld(filename);
    return true;
}

bool HomotopyViz::initWorld(QString filename) {

    std::vector< std::vector<Point2D> > conts;
    int map_width = 0, map_height = 0;
    if (mpWorld) {
        delete mpWorld;
        mpWorld = NULL;
    }

    load_map_info( filename.toStdString(), map_width, map_height, conts );   
    //std::cout << "CREATE WORLD " << map_width << " * " << map_height << std::endl;

    mpWorld = new WorldMap(map_width, map_height);
    //std::cout << "NUM OF OBS " << conts.size() << std::endl;
    if (mpWorld) {
        mpWorld->load_obstacle_info(conts);
        //std::cout << "INIT ... " << std::endl;
        mpWorld->init();

        mColors.clear();
        for( unsigned int i=0; i< mpWorld->get_obstacles().size(); i++ ) {
            mColors.push_back(QColor( rand()%255, rand()%255, rand()%255 ));
            mColors.push_back(QColor( rand()%255, rand()%255, rand()%255 ));
        }
    }
    return true;
}

void HomotopyViz::paintEvent(QPaintEvent * e) {
    QLabel::paintEvent(e);

    if (mpWorld) {

        if( mRegionIdx >= 0 ) {
            QPainter region_painter(this);
            region_painter.setRenderHint(QPainter::Antialiasing);
            QBrush region_brush( mColors[mRegionIdx] );
            region_painter.setPen(Qt::NoPen);
            SubRegionSet* p_subregion_set = mpWorld->get_subregion_set()[mRegionIdx];
            if (p_subregion_set) {
                if ( mShowSubregion == false ) {
                    QPolygon poly;
                    for( unsigned int j=0; j < p_subregion_set->m_boundary_points.size(); j++ ) {
                        double x = CGAL::to_double( p_subregion_set->m_boundary_points[j].x() );
                        double y = CGAL::to_double( p_subregion_set->m_boundary_points[j].y() );
                        poly << QPoint( x, y );
                    }
                    QPainterPath tmpPath;
                    tmpPath.addPolygon(poly);
                    region_painter.fillPath(tmpPath, region_brush);
                }
                else {                    
                    SubRegion* p_subreg = p_subregion_set->m_subregions[mSubRegionIdx];
                    if( p_subreg ) {
                        QPolygon poly;
                        for( unsigned int j=0; j < p_subreg->m_points.size(); j++ ) {
                            double x = CGAL::to_double( p_subreg->m_points[j].x() );
                            double y = CGAL::to_double( p_subreg->m_points[j].y() );
                            poly << QPoint( x, y );
                        }
                        QPainterPath tmpPath;
                        tmpPath.addPolygon(poly);
                        region_painter.fillPath(tmpPath, region_brush);

                        for( std::vector< LineSubSegment* >::iterator itLSS = p_subreg->m_neighbors.begin();
                             itLSS != p_subreg->m_neighbors.end(); itLSS++ ) {
                            LineSubSegment* p_line_subsegment = (*itLSS);

                            QPainter line_hl_painter(this);
                            QPen line_hl_pen( LINE_HIGHLIGHTED_COLOR );
                            line_hl_pen.setWidth( LINE_WIDTH_HIGHLIGHTED );
                            line_hl_painter.setPen( line_hl_pen );

                            Point2D line_hl_src = p_line_subsegment->m_subseg.source();
                            Point2D line_hl_end = p_line_subsegment->m_subseg.target();
                            double line_hl_src_x = CGAL::to_double( line_hl_src.x() );
                            double line_hl_src_y = CGAL::to_double( line_hl_src.y() );
                            double line_hl_end_x = CGAL::to_double( line_hl_end.x() );
                            double line_hl_end_y = CGAL::to_double( line_hl_end.y() );

                            line_hl_painter.drawLine( QPoint( line_hl_src_x, line_hl_src_y ),
                                                      QPoint( line_hl_end_x, line_hl_end_y ) );
                        }
                    }
                }
            }
        }

        std::vector<Obstacle*> obstacles =  mpWorld->get_obstacles();

        QPainter obstacle_painter(this);
        obstacle_painter.setRenderHint(QPainter::Antialiasing);
        QPen obstacle_pen( OBSTACLE_COLOR );
        obstacle_painter.setPen(obstacle_pen);
        for( std::vector<Obstacle*>::iterator it = obstacles.begin();
             it != obstacles.end(); it++ ) {
            Obstacle* p_obstacle = (*it);
            if (p_obstacle) {
                QPolygon poly;
                for( Polygon2D::Vertex_iterator itP=p_obstacle->m_pgn.vertices_begin();
                     itP != p_obstacle->m_pgn.vertices_end(); itP++ ) {
                    Point2D p = (*itP);
                    double p_x = CGAL::to_double( p.x() );
                    double p_y = CGAL::to_double( p.y() );
                    poly << QPoint( p_x, p_y );
                }
                obstacle_painter.drawPolygon(poly);
            }
        }

        if ( mShowSubsegment == false ) {

            QPainter alpha_painter(this);
            QPen alpha_pen( ALPHA_COLOR );
            alpha_pen.setWidth( LINE_WIDTH );
            alpha_painter.setPen( alpha_pen );

            for( std::vector<Obstacle*>::iterator it= obstacles.begin();
                 it != obstacles.end(); it++ ) {
                Obstacle* p_obstacle = (*it);
                if ( p_obstacle ) {
                    Point2D a_src = p_obstacle->mp_alpha_seg->m_seg.source();
                    Point2D a_end = p_obstacle->mp_alpha_seg->m_seg.target();
                    double a_src_x = CGAL::to_double( a_src.x() );
                    double a_src_y = CGAL::to_double( a_src.y() );
                    double a_end_x = CGAL::to_double( a_end.x() );
                    double a_end_y = CGAL::to_double( a_end.y() );
                    alpha_painter.drawLine( QPoint( a_src_x, a_src_y ), QPoint( a_end_x, a_end_y ) );
                }
            }

            QPainter beta_painter(this);
            QPen beta_pen( BETA_COLOR );
            beta_pen.setWidth( LINE_WIDTH );
            beta_painter.setPen( beta_pen );

            for( std::vector<Obstacle*>::iterator it = obstacles.begin();
                 it != obstacles.end(); it++ ) {
                Obstacle* p_obstacle = (*it);
                if ( p_obstacle ) {
                    Point2D b_src = p_obstacle->mp_beta_seg->m_seg.source();
                    Point2D b_end = p_obstacle->mp_beta_seg->m_seg.target();
                    double b_src_x = CGAL::to_double( b_src.x() );
                    double b_src_y = CGAL::to_double( b_src.y() );
                    double b_end_x = CGAL::to_double( b_end.x() );
                    double b_end_y = CGAL::to_double( b_end.y() );
                    beta_painter.drawLine( QPoint( b_src_x, b_src_y ), QPoint( b_end_x, b_end_y ) );
                }
            }
        }
        else {
            QPainter a_subseg_painter(this);
            QPen a_subseg_pen( ALPHA_COLOR );
            a_subseg_pen.setWidth( LINE_WIDTH );
            a_subseg_painter.setPen( a_subseg_pen );
            for( std::vector<Obstacle*>::iterator it = obstacles.begin();
                 it != obstacles.end(); it++ ) {
                Obstacle* p_obstacle = (*it);
                if ( p_obstacle ) {
                    //std::cout << "OBS " << p_obstacle->get_index() << " ALPHA:" << p_obstacle->mp_alpha_seg->m_subsegs.size() << std::endl;
                    for( std::vector< LineSubSegment* >::iterator itap = p_obstacle->mp_alpha_seg->m_subsegs.begin();
                         itap != p_obstacle->mp_alpha_seg->m_subsegs.end(); itap++ ) {
                        LineSubSegment* p_subseg_a = (*itap);
                        double a_src_x = CGAL::to_double( p_subseg_a->m_subseg.source().x() );
                        double a_src_y = CGAL::to_double( p_subseg_a->m_subseg.source().y() );
                        double a_end_x = CGAL::to_double( p_subseg_a->m_subseg.target().x() );
                        double a_end_y = CGAL::to_double( p_subseg_a->m_subseg.target().y() );
                        //std::cout << p_subseg_a << std::endl;
                        //std::cout << p_subseg_a->get_name() << " (" << a_src_x << "," << a_src_y << ") (" << a_end_x << "," << a_end_y << ")" << std::endl;
                        a_subseg_painter.drawLine( QPoint( a_src_x , a_src_y ), QPoint( a_end_x , a_end_y ));
                    }
                }
            }

            QPainter b_subseg_painter(this);
            QPen b_subseg_pen( BETA_COLOR );
            b_subseg_pen.setWidth( LINE_WIDTH );
            b_subseg_painter.setPen( b_subseg_pen );
            for( std::vector<Obstacle*>::iterator it = obstacles.begin();
                 it != obstacles.end(); it++ ) {
                Obstacle* p_obstacle = (*it);
                if ( p_obstacle ) {
                    //std::cout << "OBS " << p_obstacle->get_index() << " BETA:" << p_obstacle->mp_beta_seg->m_subsegs.size() << std::endl;
                    for( std::vector< LineSubSegment* >::iterator itbp = p_obstacle->mp_beta_seg->m_subsegs.begin();
                         itbp != p_obstacle->mp_beta_seg->m_subsegs.end(); itbp++ ) {
                        LineSubSegment* p_subseg_b = (*itbp);
                        double b_src_x = CGAL::to_double( p_subseg_b->m_subseg.source().x() );
                        double b_src_y = CGAL::to_double( p_subseg_b->m_subseg.source().y() );
                        double b_end_x = CGAL::to_double( p_subseg_b->m_subseg.target().x() );
                        double b_end_y = CGAL::to_double( p_subseg_b->m_subseg.target().y() );
                        //std::cout << p_subseg_b << std::endl;
                        //std::cout << p_subseg_b->get_name() << " (" << b_src_x << "," << b_src_y << ") (" << b_end_x << "," << b_end_y << ")" << std::endl;
                        b_subseg_painter.drawLine( QPoint( b_src_x , b_src_y ), QPoint( b_end_x , b_end_y ));
                    }
                }
            }
        }

        QPainter cp_painter(this);
        QPen cp_pen( CENTER_POINT_COLOR );
        cp_pen.setWidth( POINT_SIZE );
        cp_painter.setPen( cp_pen );
        double cp_x = CGAL::to_double( mpWorld->get_central_point().x() );
        double cp_y = CGAL::to_double( mpWorld->get_central_point().y() );
        cp_painter.drawPoint( QPoint( cp_x , cp_y ) );

        QPainter bk_painter(this);
        QPen bk_pen( BK_COLOR );
        bk_pen.setWidth( POINT_SIZE );
        bk_painter.setPen( bk_pen );
        for( std::vector<Obstacle*>::iterator it = obstacles.begin();
             it != obstacles.end(); it++ ) {
            Obstacle* p_obstacle = (*it);
            if ( p_obstacle ) {
                double bk_x = CGAL::to_double( p_obstacle->m_bk.x() );
                double bk_y = CGAL::to_double( p_obstacle->m_bk.y() );
                bk_painter.drawPoint( QPoint( bk_x , bk_y ) );
            }
        }

        QPainter intsec_painter(this);
        QPen intsec_pen( INTERSECTION_COLOR );
        intsec_pen.setWidth( POINT_SIZE );
        intsec_painter.setPen( intsec_pen );
        for( std::vector<Obstacle*>::iterator it = obstacles.begin();
             it != obstacles.end(); it++ ) {
            Obstacle* p_obstacle = (*it);
            if ( p_obstacle ) {
                for( std::vector< IntersectionPoint >::iterator itap = p_obstacle->m_alpha_intersection_points.begin();
                     itap != p_obstacle->m_alpha_intersection_points.end(); itap++ ) {
                    IntersectionPoint alpha_intsec = (*itap);
                    double alpha_intsec_x = CGAL::to_double( alpha_intsec.m_point.x() );
                    double alpha_intsec_y = CGAL::to_double( alpha_intsec.m_point.y() );
                    intsec_painter.drawPoint( QPoint( alpha_intsec_x , alpha_intsec_y ) );
                }
                for( std::vector< IntersectionPoint >::iterator itbp = p_obstacle->m_beta_intersection_points.begin();
                     itbp != p_obstacle->m_beta_intersection_points.end(); itbp++ ) {
                    IntersectionPoint beta_intsec = (*itbp);
                    double beta_intsec_x = CGAL::to_double( beta_intsec.m_point.x() );
                    double beta_intsec_y = CGAL::to_double( beta_intsec.m_point.y() );
                    intsec_painter.drawPoint( QPoint( beta_intsec_x, beta_intsec_y ) );
                }
            }
        }

        QPainter text_painter(this);
        QPen text_pen( TEXT_COLOR );
        text_painter.setPen(text_pen);
        for( std::vector<Obstacle*>::iterator it = obstacles.begin();
             it != obstacles.end(); it++ ) {
            Obstacle* p_obstacle = (*it);
            if( p_obstacle ) {
                int c_x = (p_obstacle->m_pgn.bbox().xmax() + p_obstacle->m_pgn.bbox().xmin() )/2;
                int c_y = (p_obstacle->m_pgn.bbox().ymax() + p_obstacle->m_pgn.bbox().ymin() )/2;
                text_painter.drawText( c_x, c_y, QString::number(p_obstacle->get_index()) );
            }
        }

        QPainter draw_line_painter(this);
        QPen draw_line_pen( DRAWING_LINE_COLOR );
        draw_line_pen.setWidth( LINE_WIDTH );
        draw_line_painter.setPen(draw_line_pen);
        if( mPoints.size() > 1 ) {
            for( unsigned int pi = 0; pi < mPoints.size()-1 ; pi ++ ) {
                draw_line_painter.drawLine( mPoints[pi], mPoints[pi+1] );    
            }
        }
    }
}

void HomotopyViz::prevRegion() {
    if( mpWorld ) {
        if ( mRegionIdx >= 0 ) {
            mRegionIdx--;
            mSubRegionIdx = 0;
        }
        else {
            mRegionIdx = static_cast<int>(mpWorld->get_subregion_set().size())-1;
            mSubRegionIdx = 0;
        }
    }
}

void HomotopyViz::nextRegion() {
    if( mpWorld ) {
        if ( mRegionIdx < static_cast<int>(mpWorld->get_subregion_set().size())-1 ) {
            mRegionIdx++;
            mSubRegionIdx = 0;
        }
        else {
            mRegionIdx = -1;
            mSubRegionIdx = 0;
        }
    }
}

void HomotopyViz::prevSubregion() {
    if ( mpWorld ) {
        if ( mRegionIdx >= 0 && mRegionIdx < static_cast<int>(mpWorld->get_subregion_set().size()) ) {
            SubRegionSet* p_subregions = mpWorld->get_subregion_set() [mRegionIdx];
            int sub_num = static_cast<int>( p_subregions->m_subregions.size() );
            if ( mSubRegionIdx > 0) {
                mSubRegionIdx --;
            }
            else{
                mSubRegionIdx = sub_num - 1;
            }
        }
    }
}

void HomotopyViz::nextSubregion() {
    if ( mpWorld ) {
        if ( mRegionIdx >= 0 && mRegionIdx < static_cast<int>(mpWorld->get_subregion_set().size()) ) {
            SubRegionSet* p_subregions = mpWorld->get_subregion_set() [mRegionIdx];
            int sub_num = static_cast<int>( p_subregions->m_subregions.size() );
            if ( mSubRegionIdx < sub_num-1) {
                mSubRegionIdx ++;
            }
            else{
                mSubRegionIdx = 0;
            }
        }
    }
}

bool HomotopyViz::save( QString filename ) {
    if( mpWorld ) {
        mpWorld->to_xml(filename.toStdString());
        return true;
    }
    return false;
}

bool HomotopyViz::load( QString filename ) {
    if ( mpWorld == NULL) {
        mpWorld = new WorldMap();
    }

    mpWorld->from_xml(filename.toStdString());

    QPixmap emptyPix( mpWorld->get_width(), mpWorld->get_height() );
    emptyPix.fill(QColor("white"));
    std::cout << " EMPTY PIX " << emptyPix.width() << " * " << emptyPix.height() << std::endl;
    //setPixmap(pix);
    setPixmap(emptyPix);

    mpWorld->init(false);
    mColors.clear();
    for( unsigned int i=0; i< mpWorld->get_obstacles().size(); i++ ) {
        mColors.push_back(QColor( rand()%255, rand()%255, rand()%255 ));
        mColors.push_back(QColor( rand()%255, rand()%255, rand()%255 ));
    }
    repaint();

    return true;
}

void HomotopyViz::mousePressEvent( QMouseEvent * event ) {
    //std::cout << "mousePressEvent" << std::endl;
    if ( event->button() == Qt::LeftButton ) {
        mDragging = true;
        mPoints.clear();    
    }
}

void HomotopyViz::mouseMoveEvent( QMouseEvent * event ) {
    //std::cout << "mouseMoveEvent" << mPoints.size() << std::endl;
    if ( mDragging == true ) {
        //std::cout << event->x() << " " << event->y() << std::endl;
        QPoint point( event->x(), event->y() );
        mPoints.push_back( point );
        repaint();
    }
}

void HomotopyViz::mouseReleaseEvent( QMouseEvent * event ){
    //std::cout << "mouseReleaseEvent" << std::endl;
    if ( event->button() == Qt::LeftButton ) {
        mDragging = false;
    }
}

