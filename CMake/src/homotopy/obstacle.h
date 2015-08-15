#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <vector>
#include "world_datatype.h"
#include "line_subsegment.h"

class WorldMap;

class Obstacle {

public:

    Obstacle(std::vector<Point2D> points, int index, WorldMap* world);
    ~Obstacle();

    Point2D sample_position();
    double distance_to_bk( Point2D& point );


    std::vector<Point2D> m_points;
    Polygon2D m_pgn;
    Point2D m_bk;
    std::vector<Segment2D> m_segments;

    Ray2D m_alpha_ray;
    Ray2D m_beta_ray;

    LineSubSegmentSet* mp_alpha_seg;
    LineSubSegmentSet* mp_beta_seg;

    std::vector<IntersectionPoint> m_alpha_intersection_points;
    std::vector<IntersectionPoint> m_beta_intersection_points;

    WorldMap* get_world() { return _p_world; }

    double m_dist_bk2cp;

protected:

    int _index;
    int _min_x;
    int _min_y;
    int _max_x;
    int _max_y;
    Point2D _centroid;
    WorldMap* _p_world;

};

#endif // OBSTACLE_H
