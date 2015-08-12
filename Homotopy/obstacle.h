#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <vector>
#include "world_datatype.h"

class Obstacle {


public:
    Obstacle(std::vector<Point2D> points, int index = 0);

    Point2D sample_position();

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

    Segment2D m_alpha_seg;
    Segment2D m_beta_seg;

};

#endif // OBSTACLE_H
