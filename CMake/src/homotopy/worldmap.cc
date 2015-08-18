#include <sstream>
#include "opencv2/core/core.hpp"
#include <CGAL/intersections.h>
#include <CGAL/Polygon_2_algorithms.h>
#include "worldmap.h"


WorldMap::WorldMap( int width, int height ) {
    _map_width = width;
    _map_height = height;

    _sample_width_scale = _map_width/5;
    _sample_height_scale = _map_width/5;

    _obstacles.clear();
    _obs_bk_pair_lines.clear();

    _central_point = Point2D(width/2, height/2);
}

WorldMap::~WorldMap() {

    for( unsigned int i=0; i < _obstacles.size(); i++ ) {
        Obstacle* po = _obstacles[i];
        delete po;
        po = NULL;
    }
    _obstacles.clear();
    _line_segments.clear();

}

bool WorldMap::load_obstacle_info( std::vector< std::vector<Point2D> > polygons ) {
    _obstacles.clear();
    int obs_idx = 0;
    for( std::vector< std::vector<Point2D> >::iterator it=polygons.begin(); it!=polygons.end(); it++ ) {
        std::vector<Point2D> points = (*it);
        Obstacle* p_obs = new Obstacle(points, obs_idx, this);
        obs_idx ++;
        _obstacles.push_back(p_obs);
    }

    return true;
}

bool WorldMap::init() {

    // select random point for each obstacle
    for( std::vector<Obstacle*>::iterator it = _obstacles.begin(); it != _obstacles.end(); it++ ) {
        Obstacle * p_obstacle = (*it);
        p_obstacle->m_bk = p_obstacle->sample_position();
    }

    _obs_bk_pair_lines.clear();
    for( unsigned int i=0; i < _obstacles.size(); i++ ) {
        for( unsigned int j=i+1; j < _obstacles.size(); j++ ) {
            Line2D pline(_obstacles[i]->m_bk, _obstacles[j]->m_bk);
            _obs_bk_pair_lines.push_back(pline);
        }
    }

    // select central point c
    bool found_cp = false;
    while( found_cp == false ) {
        if ( (false == _is_in_obstacle(_central_point)) && (false == _is_in_obs_bk_lines(_central_point)) ) {
            found_cp = true;
        }
        else {
            float x_ratio = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
            float y_ratio = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
            int cp_x = static_cast<int>( x_ratio*static_cast<float>(_sample_width_scale) ) + _map_width /2;
            int cp_y = static_cast<int>( y_ratio*static_cast<float>(_sample_height_scale) ) + _map_height /2;
            _central_point = Point2D(cp_x, cp_y);
        }
    }

    for( std::vector<Obstacle*>::iterator it = _obstacles.begin(); it != _obstacles.end(); it++ ) {
        Obstacle * p_obstacle = (*it);
        p_obstacle->m_dist_bk2cp = p_obstacle->distance_to_bk(_central_point);
    }

    // init four boundary line
    _boundary_lines.clear();
    _x_min_line = Segment2D(Point2D(0,0), Point2D(_map_width-1,0));
    _y_min_line = Segment2D(Point2D(0,0), Point2D(0,_map_height-1));
    _x_max_line = Segment2D(Point2D(0,_map_height-1), Point2D(_map_width-1,_map_height-1));
    _y_max_line = Segment2D(Point2D(_map_width-1,0), Point2D(_map_width-1,_map_height-1));
    _boundary_lines.push_back(_x_min_line);
    _boundary_lines.push_back(_y_max_line);
    _boundary_lines.push_back(_x_max_line);
    _boundary_lines.push_back(_y_min_line);

    // init lines from center point to four corners

    // init alpha and beta segments
    for( std::vector<Obstacle*>::iterator it=_obstacles.begin(); it!=_obstacles.end(); it++) {
        Obstacle* p_obstacle = (*it);
        p_obstacle->m_alpha_ray = Ray2D( p_obstacle->m_bk, _central_point );
        p_obstacle->m_beta_ray = Ray2D( p_obstacle->m_bk, Point2D(2*p_obstacle->m_bk.x()-_central_point.x(), 2*p_obstacle->m_bk.y()-_central_point.y()) );

        Point2D * p_a_pt = _find_intersection_with_boundary( p_obstacle->m_alpha_ray );
        Point2D * p_b_pt = _find_intersection_with_boundary( p_obstacle->m_beta_ray );

        if ( p_a_pt ) {
            p_obstacle->mp_alpha_seg = new LineSubSegmentSet( p_obstacle->m_bk, *p_a_pt, LINE_TYPE_ALPHA, p_obstacle->m_alpha_ray.direction(), p_obstacle );
            _line_segments.push_back(p_obstacle->mp_alpha_seg);
        }
        if ( p_b_pt ) {
            p_obstacle->mp_beta_seg = new LineSubSegmentSet( p_obstacle->m_bk, *p_b_pt, LINE_TYPE_BETA, p_obstacle->m_beta_ray.direction(), p_obstacle );
            _line_segments.push_back(p_obstacle->mp_beta_seg);
        }
    }

    std::sort(_line_segments.begin(), _line_segments.end());

    return true;
}

bool WorldMap::init_segments() {

    for( std::vector<Obstacle*>::iterator it=_obstacles.begin(); it!=_obstacles.end(); it++) {
        Obstacle* p_obstacle = (*it);
        p_obstacle->m_alpha_intersection_points.clear();
        p_obstacle->m_beta_intersection_points.clear();

        for( std::vector<Obstacle*>::iterator itr=_obstacles.begin(); itr!=_obstacles.end(); itr++) {
            Obstacle* p_ref_obstacle = (*itr);
            // check alpha_seg with obstacles
            std::vector< Point2D > a_ints = _intersect(p_obstacle->mp_alpha_seg->m_seg, p_ref_obstacle->m_segments);
            std::vector< Point2D > b_ints = _intersect(p_obstacle->mp_beta_seg->m_seg, p_ref_obstacle->m_segments);
            for( std::vector< Point2D >::iterator itp = a_ints.begin(); itp != a_ints.end(); itp++ ) {
                Point2D p = (*itp);
                IntersectionPoint ip;
                ip.m_point = p;
                ip.m_dist_to_bk = p_obstacle->distance_to_bk(p);
                p_obstacle->m_alpha_intersection_points.push_back(ip);
            }
            for( std::vector< Point2D >::iterator itp = b_ints.begin(); itp != b_ints.end(); itp++ ) {
                Point2D p = (*itp);
                IntersectionPoint ip;
                ip.m_point = p;
                ip.m_dist_to_bk = p_obstacle->distance_to_bk(p);
                p_obstacle->m_beta_intersection_points.push_back(ip);
            }
        }

        std::sort(p_obstacle->m_alpha_intersection_points.begin(), p_obstacle->m_alpha_intersection_points.end());
        std::sort(p_obstacle->m_beta_intersection_points.begin(), p_obstacle->m_beta_intersection_points.end());

        p_obstacle->mp_alpha_seg->load( p_obstacle->m_alpha_intersection_points );
        p_obstacle->mp_beta_seg->load( p_obstacle->m_beta_intersection_points );

        /*
        std::cout << *p_obstacle << std::endl;
        std::cout << "ALPHA " << std::endl;
        for( unsigned int i = 0; i < p_obstacle->m_alpha_intersection_points.size(); i++ ) {
            std::cout << p_obstacle->m_alpha_intersection_points[i];
        }
        std::cout << p_obstacle->mp_alpha_seg;
        std::cout << "BETA " << std::endl;
        for( unsigned int i = 0; i < p_obstacle->m_beta_intersection_points.size(); i++ ) {
            std::cout << p_obstacle->m_beta_intersection_points[i];
        }
        std::cout << p_obstacle->mp_beta_seg;
        */

    }
    return true;
}

bool WorldMap::_is_in_obs_bk_lines(Point2D point) {

    for( std::vector<Line2D>::iterator it = _obs_bk_pair_lines.begin(); it != _obs_bk_pair_lines.end(); it++ ) {
        Line2D bk_line = (*it);
        if ( bk_line.has_on( point )==true ) {
            return true;
        }
    }
    return false;
}

Point2D* WorldMap::_find_intersection_with_boundary(Ray2D ray) {

    for(std::vector<Segment2D>::iterator it=_boundary_lines.begin(); it!=_boundary_lines.end(); it++) {
        Segment2D seg = (*it);
        CGAL::Object result = intersection(seg, ray);
        Point2D* p = new Point2D();
        if ( CGAL::assign(*p, result) ) {
            return p;
        }
        if(p) {
            delete p;
            p = NULL;
        }
    }
    return NULL;
}

bool WorldMap::_is_in_obstacle( Point2D point ) {
    for( std::vector<Obstacle*>::iterator it = _obstacles.begin(); it != _obstacles.end(); it++ ) {
        Obstacle * p_obstacle = (*it);
        if ( CGAL::ON_UNBOUNDED_SIDE != p_obstacle->m_pgn.bounded_side( point ) ) {
            return true;
        }
    }
    return false;
}

std::vector<Point2D> WorldMap::_intersect( Segment2D seg, std::vector<Segment2D> segments ) {
    std::vector<Point2D> points;
    for(std::vector<Segment2D>::iterator it=segments.begin(); it!=segments.end(); it++) {
        Segment2D bound = (*it);
        CGAL::Object result = intersection(seg, bound);
        Point2D p;
        if ( CGAL::assign(p, result) ) {
            /*
            if ( points.size()>1 && squared_distance(p, points[points.size()-1]) <= 1.0 ) {
                continue;
            }*/
            points.push_back(p);

        }
    }
    return points;
}


void WorldMap::to_xml( const std::string& filename )const {
    xmlDocPtr doc = xmlNewDoc( ( xmlChar* )( "1.0" ) );
    xmlNodePtr root = xmlNewDocNode( doc, NULL, ( xmlChar* )( "root" ), NULL );
    xmlDocSetRootElement( doc, root );
    to_xml( doc, root );
    xmlSaveFormatFileEnc( filename.c_str(), doc, "UTF-8", 1 );
    xmlFreeDoc( doc );
    return;
}

void WorldMap::to_xml( xmlDocPtr doc, xmlNodePtr root )const {
    xmlNodePtr world_node = xmlNewDocNode( doc, NULL, ( xmlChar* )( "world" ), NULL );
    std::stringstream width_str, height_str;
    width_str << _map_width;
    height_str << _map_height;
    xmlNewProp( world_node, ( const xmlChar* )( "width" ), ( const xmlChar* )( width_str.str().c_str() ) );
    xmlNewProp( world_node, ( const xmlChar* )( "height" ), ( const xmlChar* )( height_str.str().c_str() ) );
    xmlAddChild( root, world_node );

    return;
}

void WorldMap::from_xml( const std::string& filename ) {

}

void WorldMap::from_xml( xmlNodePtr root ) {

}

std::ostream& operator<<( std::ostream& out, const WorldMap& other ) {

    out << "Size[" << other.get_width() << "*" << other.get_height() << "]  " << std::endl;
    for( unsigned int i=0; i < other.get_obstacles().size(); i++ ) {
        out << other.get_obstacles()[i] << std::endl;
    }
    return out;
}
