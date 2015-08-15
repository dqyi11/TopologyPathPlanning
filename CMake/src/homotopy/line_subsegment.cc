#include <CGAL/squared_distance_2.h>
#include "line_subsegment.h"
#include "obstacle.h"
#include "worldmap.h"

LineSubSegment::LineSubSegment( Point2D& pos_a, Point2D& pos_b, LineSubSegmentSet* p_subseg_set, unsigned int index, bool is_connected_to_central_point ) {
    _index = index;
    m_is_connected_to_central_point = is_connected_to_central_point;
    m_subseg = Segment2D( pos_a, pos_b );
    _p_subseg_set = p_subseg_set;
    if( _p_subseg_set ) {
        _p_subseg_set->m_subsegs.push_back(this);
    }
}

LineSubSegmentSet::LineSubSegmentSet( Point2D& pos_a, Point2D& pos_b, unsigned int type, Direction2D direction, Obstacle* p_obstacle ) {

    m_seg = Segment2D(pos_a, pos_b);
    m_type = type;
    m_direction = direction;
    _p_obstacle = p_obstacle;
    m_subsegs.clear();
}

LineSubSegmentSet::~LineSubSegmentSet() {

}

std::string LineSubSegmentSet::type_to_std_string ( const unsigned int& type ) {
    switch( type ) {
    case( LINE_TYPE_ALPHA ):
        return "A";
    case( LINE_TYPE_BETA ):
        return "B";
    case( LINE_TYPE_UNKNOWN ):
    default:
        return "na";
    }
}

unsigned int LineSubSegmentSet::type_from_std_string ( const std::string& type_str ) {
    for( unsigned int i = 0; i < NUM_LINE_TYPE; i++ ){
      if( type_str == LineSubSegmentSet::type_to_std_string( i ) ){
        return i;
      }
    }
    return LINE_TYPE_UNKNOWN;
}

bool LineSubSegmentSet::load( std::vector<IntersectionPoint>& intersections ) {

    if( _p_obstacle == NULL ) {
        return false;
    }
    if( _p_obstacle->get_world() == NULL ) {
        return false;
    }
    if( m_type == LINE_TYPE_ALPHA ) {
        double cp_dist = CGAL::squared_distance(_p_obstacle->m_bk, _p_obstacle->get_world()->get_central_point() );
        bool pass_central_point = false;
        unsigned int idx = 0;
        for( unsigned int i = 1; i < intersections.size()-1; i+=2 ) {
            IntersectionPoint sec1 = intersections[i];
            IntersectionPoint sec2 = intersections[i+1];

            if ( pass_central_point == false ) {
                if ( sec1.m_dist_to_bk <= _p_obstacle->m_dist_bk2cp && sec2.m_dist_to_bk >= _p_obstacle->m_dist_bk2cp ) {
                    LineSubSegment* p_subseg = new LineSubSegment( sec1.m_point , _p_obstacle->get_world()->get_central_point(), this, idx, true );
                    m_subsegs.push_back(p_subseg);
                    idx += 1;

                    pass_central_point = true;

                    LineSubSegment* p_subseg2 = new LineSubSegment( _p_obstacle->get_world()->get_central_point(), sec2.m_point, this, idx, true );
                    m_subsegs.push_back(p_subseg2);
                    idx += 1;

                }
            }
            else {
                LineSubSegment* p_subseg = new LineSubSegment( sec1.m_point, sec2.m_point, this, idx );
                m_subsegs.push_back(p_subseg);
                idx += 1;
            }
        }

        if ( intersections[intersections.size()-1].m_dist_to_bk > _p_obstacle->m_dist_bk2cp ) {
            Point2D end_point = static_cast<Point2D>(m_seg.target());
            LineSubSegment* p_subseg = new LineSubSegment( intersections[intersections.size()-1].m_point, end_point, this, idx );
            m_subsegs.push_back(p_subseg);
            idx += 1;
        }
        else {
            LineSubSegment* p_subseg = new LineSubSegment( intersections[intersections.size()-1].m_point, _p_obstacle->get_world()->get_central_point(), this, idx, true );
            m_subsegs.push_back(p_subseg);
            idx += 1;

            pass_central_point = true;

            Point2D end_point = static_cast<Point2D>(m_seg.target());
            LineSubSegment* p_subseg2 = new LineSubSegment( _p_obstacle->get_world()->get_central_point(), end_point, this, idx, true );
            m_subsegs.push_back(p_subseg2);
            idx += 1;
        }
    }
    else if( m_type == LINE_TYPE_BETA ) {
        unsigned int idx = 0;
        for( unsigned int i = 1; i < intersections.size()-1; i+=2 ) {
            IntersectionPoint sec1 = intersections[i];
            IntersectionPoint sec2 = intersections[i+1];

            LineSubSegment* p_subseg = new LineSubSegment( sec1.m_point, sec2.m_point, this, idx );
            m_subsegs.push_back(p_subseg);
            idx += 1;
        }
        Point2D end_point = static_cast<Point2D>(m_seg.target());
        LineSubSegment* p_subseg = new LineSubSegment( intersections[intersections.size()-1].m_point, end_point, this, idx );
        m_subsegs.push_back(p_subseg);
        idx += 1;

    }else {
        return false;
    }
    return true;
}
