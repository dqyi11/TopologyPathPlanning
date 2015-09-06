#include <string>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "world_datatype.h"

bool load_map_info( std::string filename, int& width, int& height, std::vector< std::vector<Point2D> >& obstacles ) {
    obstacles.clear();

    cv::Mat src;
    src = cv::imread(filename,  CV_LOAD_IMAGE_GRAYSCALE);
    cv::threshold(src, src, 200, 255, cv::THRESH_BINARY_INV);
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( src, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE );
    width = src.cols;
    height = src.rows;
    for( unsigned int i=0; i<contours.size(); i++ ) {
        std::vector<Point2D> cont;
        for( std::vector<cv::Point>::iterator it=contours[i].begin(); it!=contours[i].end(); it++ ) {
           cv::Point p = (cv::Point)(*it);
           Point2D ip(p.x, p.y);
           cont.push_back(ip);
        }
        obstacles.push_back(cont);
    }
    return true;
}
