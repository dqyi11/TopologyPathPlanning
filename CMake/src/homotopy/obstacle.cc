#include <stdlib.h>
#include <CGAL/centroid.h>
#include <CGAL/squared_distance_2.h>
#include "obstacle.h"

Obstacle::Obstacle(std::vector<Point2D> points, int index ){
    _index = index;

    m_points.clear();
    m_segments.clear();
    for( std::vector<Point2D>::iterator it = points.begin(); it != points.end(); it++ ) {
        Point2D pos = (*it);
        m_points.push_back(pos);
        m_pgn.push_back(pos);
    }

    for( unsigned int i=0; i< m_points.size(); i++ ) {
        if( i < m_points.size()-1 ) {
            Segment2D seg( m_points[i], m_points[i+1] );
            m_segments.push_back(seg);
        }
        else {
            Segment2D seg( m_points[i], m_points[0] );
            m_segments.push_back(seg);
        }
    }

    _min_x = m_pgn.bbox().xmin();
    _min_y = m_pgn.bbox().ymin();
    _max_x = m_pgn.bbox().xmax();
    _max_y = m_pgn.bbox().ymax();

    _centroid = Point2D( (_min_x + _max_x)/2 , (_min_y + _max_y)/2 );
}

Obstacle::~Obstacle() {
    m_points.clear();
    m_segments.clear();
}

double Obstacle::distance_to_bk( Point2D& point ) {
    double dist = 0.0;
    dist = squared_distance( m_bk , point );
    return dist;
}

Point2D Obstacle::sample_position() {

    bool found = false;
    while (found == false) {
        float x_ratio = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
        float y_ratio = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
        int rnd_x = static_cast<int>(x_ratio*(_max_x - _min_x)) + _min_x;
        int rnd_y = static_cast<int>(y_ratio*(_max_y - _min_y)) + _min_y;

        Point2D rnd_point(rnd_x, rnd_y);
        if ( CGAL::ON_BOUNDED_SIDE == m_pgn.bounded_side( rnd_point ) ) {
            return rnd_point;
        }
    }
    return _centroid;
}

