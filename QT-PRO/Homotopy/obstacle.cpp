#include <stdlib.h>
#include <CGAL/centroid.h>
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

    for( int i=0; i< m_points.size(); i++ ) {
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

Point2D Obstacle::sample_position() {

    bool found = false;
    while (found == false) {
        int rnd_x = (int)(rand()*(_max_x - _min_x) + _min_x);
        int rnd_y = (int)(rand()*(_max_x - _min_x) + _min_x);

        Point2D rnd_point(rnd_x, rnd_y);
        if (m_pgn.bounded_side( rnd_point ) == CGAL::ON_BOUNDED_SIDE ) {
            return rnd_point;
        }
    }
    return _centroid;
}
