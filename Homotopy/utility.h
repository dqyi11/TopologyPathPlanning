#ifndef UTILITY_H
#define UTILITY_H

#include <list>
#include "opencv2/core/core.hpp"
#include "world_datatype.h"

std::list<std::vector<Point2D>> get_obstacles_polygon(std::sting filename) {

    cv::Mat src, gray;
    src = imread(filename, cv::CV_LOAD_IMAGE_COLOR);
    cv::cvtColor(src, gray, CV_BGR2GRAY);
    cv::threshold(gray, gray, 200, 255, cv::THRESH_BINARY_INV);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( gray, contours, hierarchy, cv::CV_RETR_CCOMP, cv::CV_CHAIN_APPROX_SIMPLE );

    std::list<std::vector<POS2D>> conts;
    for( int i=0; i<contours.size(); i++ ) {
        std::vector<int*> cont;
        for( std::vector<cv::Point>::iterator it = contours.begin(); it!=contours.end(); it++ ) {
           cv::Point p = (cv::Point)(*it);
           Point2D ip(p.x, p.y);
           cont.push_back(ip);
        }
        conts.push_back(cont);
    }
    return conts;
}

#endif // UTILITY_H
