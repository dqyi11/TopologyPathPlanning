#include <cstdlib>
#include <QtGui>
#include <QFile>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "homotopyviz.h"

HomotopyViz::HomotopyViz(QWidget *parent) :
    QLabel(parent) {

    mpWorld = NULL;
    mWorldWidth = 0;
    mWorldHeight = 0;
    mShowSubregion = false;
    mShowSubsegment = true;
    mRegionIdx = -1;
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
    cv::Mat src;
    src = cv::imread(filename.toStdString(),  CV_LOAD_IMAGE_GRAYSCALE);
    cv::threshold(src, src, 200, 255, cv::THRESH_BINARY_INV);
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( src, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

    for( unsigned int i=0; i<contours.size(); i++ ) {
        std::vector<Point2D> cont;
        for( std::vector<cv::Point>::iterator it=contours[i].begin(); it!=contours[i].end(); it++ ) {
           cv::Point p = (cv::Point)(*it);
           Point2D ip(p.x, p.y);
           cont.push_back(ip);
        }
        conts.push_back(cont);
    }

    if (mpWorld) {
        delete mpWorld;
        mpWorld = NULL;
    }

    std::cout << "CREATE WORLD " << src.cols << " * " << src.rows << std::endl;

    mpWorld = new WorldMap(src.cols, src.rows);
    std::cout << "NUM OF OBS " << conts.size() << std::endl;
    if (mpWorld) {
        mpWorld->load_obstacle_info(conts);
        std::cout << "INIT ... " << std::endl;
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
            QBrush region_brush(mColors[mRegionIdx]);
            region_painter.setPen(Qt::NoPen);
            SubRegionSet* p_subregion_set = mpWorld->get_subregion_set()[mRegionIdx];
            if (p_subregion_set) {
                if ( mShowSubregion == false ) {
                    QPolygon poly;
                    for( unsigned int j=0; j < p_subregion_set->m_boundary_points.size(); j++ ) {
                        poly << QPoint( p_subregion_set->m_boundary_points[j].x(), p_subregion_set->m_boundary_points[j].y() );
                    }
                    QPainterPath tmpPath;
                    tmpPath.addPolygon(poly);
                    region_painter.fillPath(tmpPath, region_brush);
                }
                else {
                    for( unsigned int k=0; k < p_subregion_set->m_subregions.size(); k++ ) {
                        SubRegion* p_subreg = p_subregion_set->m_subregions[k];
                        if( p_subreg ) {
                            QPolygon poly;
                            for( unsigned int j=0; j < p_subreg->m_points.size(); j++ ) {
                                poly << QPoint( p_subreg->m_points[j].x(), p_subreg->m_points[j].y() );
                            }
                            QPainterPath tmpPath;
                            tmpPath.addPolygon(poly);
                            region_painter.fillPath(tmpPath, region_brush);
                        }
                    }
                }
            }
        }


        std::vector<Obstacle*> obstacles =  mpWorld->get_obstacles();

        QPainter obstacle_painter(this);
        obstacle_painter.setRenderHint(QPainter::Antialiasing);
        QPen obstacle_pen(QColor(125,125,125));
        obstacle_painter.setPen(obstacle_pen);
        for( std::vector<Obstacle*>::iterator it = obstacles.begin();
             it != obstacles.end(); it++ ) {
            Obstacle* p_obstacle = (*it);
            if (p_obstacle) {
                QPolygon poly;
                for( unsigned int i = 0; i < p_obstacle->m_points.size(); i++ ) {
                    poly << QPoint( p_obstacle->m_points[i].x(), p_obstacle->m_points[i].y()) ;
                }
                obstacle_painter.drawPolygon(poly);
            }
        }

        if ( mShowSubsegment == false ) {

            QPainter alpha_painter(this);
            QPen alpha_pen(QColor(0,0,255));
            alpha_pen.setWidth(2);
            alpha_painter.setPen(alpha_pen);

            for( std::vector<Obstacle*>::iterator it= obstacles.begin();
                 it != obstacles.end(); it++ ) {
                Obstacle* p_obstacle = (*it);
                if ( p_obstacle ) {
                    Point2D a_src = p_obstacle->mp_alpha_seg->m_seg.source();
                    Point2D a_end = p_obstacle->mp_alpha_seg->m_seg.target();
                    alpha_painter.drawLine(QPoint(a_src.x(), a_src.y()), QPoint(a_end.x(), a_end.y()) );
                }
            }

            QPainter beta_painter(this);
            QPen beta_pen(QColor(0,255,0));
            beta_pen.setWidth(2);
            beta_painter.setPen(beta_pen);

            for( std::vector<Obstacle*>::iterator it = obstacles.begin();
                 it != obstacles.end(); it++ ) {
                Obstacle* p_obstacle = (*it);
                if ( p_obstacle ) {
                    Point2D b_src = p_obstacle->mp_beta_seg->m_seg.source();
                    Point2D b_end = p_obstacle->mp_beta_seg->m_seg.target();
                    beta_painter.drawLine(QPoint(b_src.x(), b_src.y()), QPoint(b_end.x(), b_end.y()) );
                }
            }
        }
        else {
            QPainter a_subseg_painter(this);
            QPen a_subseg_pen(QColor(0,0,255));
            a_subseg_pen.setWidth(2);
            a_subseg_painter.setPen(a_subseg_pen);
            for( std::vector<Obstacle*>::iterator it = obstacles.begin();
                 it != obstacles.end(); it++ ) {
                Obstacle* p_obstacle = (*it);
                if ( p_obstacle ) {
                    for( std::vector< LineSubSegment* >::iterator itap = p_obstacle->mp_alpha_seg->m_subsegs.begin();
                         itap != p_obstacle->mp_alpha_seg->m_subsegs.end(); itap++ ) {
                        LineSubSegment* p_subseg_a = (*itap);
                        a_subseg_painter.drawLine( QPoint(p_subseg_a->m_subseg.source().x(), p_subseg_a->m_subseg.source().y()),
                                                 QPoint(p_subseg_a->m_subseg.target().x(), p_subseg_a->m_subseg.target().y()));
                    }
                }
            }

            QPainter b_subseg_painter(this);
            QPen b_subseg_pen(QColor(0,255,0));
            b_subseg_pen.setWidth(2);
            b_subseg_painter.setPen(b_subseg_pen);
            for( std::vector<Obstacle*>::iterator it = obstacles.begin();
                 it != obstacles.end(); it++ ) {
                Obstacle* p_obstacle = (*it);
                if ( p_obstacle ) {
                    for( std::vector< LineSubSegment* >::iterator itbp = p_obstacle->mp_beta_seg->m_subsegs.begin();
                         itbp != p_obstacle->mp_beta_seg->m_subsegs.end(); itbp++ ) {
                        LineSubSegment* p_subseg_b = (*itbp);
                        b_subseg_painter.drawLine( QPoint(p_subseg_b->m_subseg.source().x(), p_subseg_b->m_subseg.source().y()),
                                                 QPoint(p_subseg_b->m_subseg.target().x(), p_subseg_b->m_subseg.target().y()));
                    }
                }
            }
        }

        QPainter cp_painter(this);
        QPen cp_pen(QColor(255,0,0));
        cp_pen.setWidth(4);
        cp_painter.setPen(cp_pen);
        cp_painter.drawPoint(QPoint(mpWorld->get_central_point().x(),
                                 mpWorld->get_central_point().y()));

        QPainter bk_painter(this);
        QPen bk_pen(QColor(255,140,0));
        bk_pen.setWidth(4);
        bk_painter.setPen(bk_pen);
        for( std::vector<Obstacle*>::iterator it = obstacles.begin();
             it != obstacles.end(); it++ ) {
            Obstacle* p_obstacle = (*it);
            if ( p_obstacle ) {
                bk_painter.drawPoint( QPoint(p_obstacle->m_bk.x(), p_obstacle->m_bk.y()) );
            }
        }

        QPainter intsec_painter(this);
        QPen intsec_pen(QColor(160,160,160));
        intsec_pen.setWidth(4);
        intsec_painter.setPen(intsec_pen);
        for( std::vector<Obstacle*>::iterator it = obstacles.begin();
             it != obstacles.end(); it++ ) {
            Obstacle* p_obstacle = (*it);
            if ( p_obstacle ) {
                for( std::vector< IntersectionPoint >::iterator itap = p_obstacle->m_alpha_intersection_points.begin();
                     itap != p_obstacle->m_alpha_intersection_points.end(); itap++ ) {
                    IntersectionPoint alpha_intsec = (*itap);
                    intsec_painter.drawPoint( QPoint(alpha_intsec.m_point.x(), alpha_intsec.m_point.y()) );
                }
                for( std::vector< IntersectionPoint >::iterator itbp = p_obstacle->m_beta_intersection_points.begin();
                     itbp != p_obstacle->m_beta_intersection_points.end(); itbp++ ) {
                    IntersectionPoint beta_intsec = (*itbp);
                    intsec_painter.drawPoint( QPoint(beta_intsec.m_point.x(), beta_intsec.m_point.y()) );
                }
            }
        }
    }
}

void HomotopyViz::prevRegion() {
    if( mpWorld ) {
        if ( mRegionIdx >= 0 ) {
            mRegionIdx--;
        }
        else {
            mRegionIdx = static_cast<int>(mpWorld->get_subregion_set().size())-1;
        }
    }
}

void HomotopyViz::nextRegion() {
    if( mpWorld ) {
        if ( mRegionIdx < static_cast<int>(mpWorld->get_subregion_set().size())-1 ) {
            mRegionIdx++;
        }
        else {
            mRegionIdx = -1;
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
