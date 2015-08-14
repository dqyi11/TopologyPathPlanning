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

    return true;
}

bool HomotopyViz::initWorld(QString filename) {

    std::list< std::vector<Point2D> > conts;
    cv::Mat src;
    src = cv::imread(filename.toStdString(),  CV_LOAD_IMAGE_GRAYSCALE);
    cv::threshold(src, src, 200, 255, cv::THRESH_BINARY_INV);
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( src, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

    for( int i=0; i<contours.size(); i++ ) {
        std::vector<Point2D> cont;
        for( std::vector<cv::Point>::iterator it=contours[i].begin(); it!=contours[i].end(); it++ ) {
           cv::Point p = (cv::Point)(*it);
           Point2D ip(p.x, p.y);
           cont.push_back(ip);
        }
        conts.push_back(cont);
    }

    return true;
}

void HomotopyViz::paintEvent(QPaintEvent * e) {
    QLabel::paintEvent(e);

    if (mpWorld) {
        QPainter painter(this);
        QPen redpen(QColor(255,0,0));
        redpen.setWidth(4);
        painter.setPen(redpen);
        painter.drawPoint(QPoint(mpWorld->get_central_point().x(),
                                 mpWorld->get_central_point().y()));

        QPen greenpen(QColor(124,252,0));
        greenpen.setWidth(4);
        painter.setPen(greenpen);
        for( std::vector<Obstacle*>::iterator it=mpWorld->get_obstacles().begin();
             it!=mpWorld->get_obstacles().end(); it++ ) {
            Obstacle* p_obstacle = (*it);
            painter.drawPoint( QPoint(p_obstacle->m_bk.x(), p_obstacle->m_bk.y()) );
        }



    }
}
