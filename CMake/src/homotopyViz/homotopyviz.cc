#include <QtGui>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "homotopyviz.h"

HomotopyViz::HomotopyViz(QWidget *parent) :
    QLabel(parent) {

    mpWorld = NULL;
    mWorldWidth = 0;
    mWorldHeight = 0;
}

bool HomotopyViz::loadMap( QString filename ) {

    QPixmap pix(filename);
    if( pix.isNull() == true ) {
        return false;
    }

    setPixmap(pix);
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
        //mpWorld->init_segments();
    }
    return true;
}

void HomotopyViz::paintEvent(QPaintEvent * e) {
    QLabel::paintEvent(e);

    if (mpWorld) {
        QPainter alpha_painter(this);
        QPen alpha_pen(QColor(0,0,255));
        alpha_pen.setWidth(2);
        alpha_painter.setPen(alpha_pen);

        for( std::vector<Obstacle*>::iterator it=mpWorld->get_obstacles().begin();
             it!=mpWorld->get_obstacles().end(); it++ ) {
            Obstacle* p_obstacle = (*it);
            Point2D a_src = p_obstacle->m_alpha_seg.source();
            Point2D a_end = p_obstacle->m_alpha_seg.target();
            alpha_painter.drawLine(QPoint(a_src.x(), a_src.y()), QPoint(a_end.x(), a_end.y()) );
        }

        QPainter beta_painter(this);
        QPen beta_pen(QColor(0,255,0));
        beta_pen.setWidth(2);
        beta_painter.setPen(beta_pen);

        for( std::vector<Obstacle*>::iterator it=mpWorld->get_obstacles().begin();
             it!=mpWorld->get_obstacles().end(); it++ ) {
            Obstacle* p_obstacle = (*it);
            Point2D b_src = p_obstacle->m_beta_seg.source();
            Point2D b_end = p_obstacle->m_beta_seg.target();
            beta_painter.drawLine(QPoint(b_src.x(), b_src.y()), QPoint(b_end.x(), b_end.y()) );
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
        for( std::vector<Obstacle*>::iterator it=mpWorld->get_obstacles().begin();
             it!=mpWorld->get_obstacles().end(); it++ ) {
            Obstacle* p_obstacle = (*it);
            bk_painter.drawPoint( QPoint(p_obstacle->m_bk.x(), p_obstacle->m_bk.y()) );
        }

    }
}
