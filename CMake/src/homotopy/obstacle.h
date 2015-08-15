#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <vector>
#include "world_datatype.h"
#include "line_subsegment.h"

class IntersectionPoint;

class Obstacle {

public:

    Obstacle(std::vector<Point2D> points, int index = 0);
    ~Obstacle();

    Point2D sample_position();
    double distance_to_bk( Point2D& point );

    int _index;
    std::vector<Point2D> m_points;
    Polygon2D m_pgn;
    Point2D _centroid;
    Point2D m_bk;
    std::vector<Segment2D> m_segments;

    int _min_x;
    int _min_y;
    int _max_x;
    int _max_y;

    Ray2D m_alpha_ray;
    Ray2D m_beta_ray;

    LineSubSegmentSet* mp_alpha_seg;
    LineSubSegmentSet* mp_beta_seg;

    std::vector<IntersectionPoint> m_alpha_intersection_points;
    std::vector<IntersectionPoint> m_beta_intersection_points;

};

class IntersectionPoint {
public:
    Point2D m_point;
    double  m_dist_to_bk;

    bool operator<(const  IntersectionPoint& other) const {
        return ( m_dist_to_bk < other.m_dist_to_bk );
    }
};



#endif // OBSTACLE_H
