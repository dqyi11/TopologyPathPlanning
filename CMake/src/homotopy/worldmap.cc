#include "opencv2/core/core.hpp"
#include <CGAL/intersections.h>
#include <CGAL/Polygon_2_algorithms.h>
#include "worldmap.h"

#define NO_OBSTACLE_PIX    255
#define OBSTACLE_PIX       0
#define OBSTACLE_THRESHOLD 200

WorldMap::WorldMap( int width, int height ) {
    _map_width = width;
    _map_height = height;

    _sample_width_scale = _map_width/5;
    _sample_height_scale = _map_width/5;
    _central_point = Point2D(width/2, height/2);

    _pp_obstalce_info = new int*[_map_width];
    for( int i=0; i<_map_width; i++ ) {
        _pp_obstalce_info[i] = new int[_map_height];
        for ( int j=0; j<_map_height; i++ ) {
            _pp_obstalce_info[i][j] = NO_OBSTACLE_PIX;
        }
    }
    _obstacles.clear();
    _obs_bk_pair_lines.clear();
}

bool WorldMap::load_obstalce_info( std::vector< std::vector<Point2D> > polygons ) {
    _obstacles.clear();
    int obs_idx = 0;
    for( std::vector< std::vector<Point2D> >::iterator it=polygons.begin(); it!=polygons.end(); it++ ) {
        std::vector<Point2D> points = (*it);
        Obstacle* p_obs = new Obstacle(points, obs_idx);
        obs_idx ++;
    }

    for( int i=0; i<_map_width; i++ ) {
        _pp_obstalce_info[i] = new int[_map_height];
        for ( int j=0; j<_map_height; i++ ) {
            if ( is_in_obstacle( Point2D(i, j) ) ) {
                _pp_obstalce_info[i][j] = NO_OBSTACLE_PIX;
            }
            else {
                _pp_obstalce_info[i][j] = OBSTACLE_PIX;
            }
        }
    }
}

bool WorldMap::init() {

    // select random point for each obstacle
    for( std::vector<Obstacle*>::iterator it = _obstacles.begin(); it != _obstacles.end(); it++ ) {
        Obstacle * p_obstacle = (*it);
        p_obstacle->m_bk = p_obstacle->sample_position();
    }

    _obs_bk_pair_lines.clear();
    for( int i=0; i < _obstacles.size(); i++ ) {
        for( int j=i+1; j < _obstacles.size(); j++ ) {
            Line2D pline(_obstacles[i]->m_bk, _obstacles[j]->m_bk);
            _obs_bk_pair_lines.push_back(pline);
        }
    }

    // select central point c
    bool found_cp = false;
    while( found_cp == false ) {
        if ( (false == _is_in_obstacle(_central_point)) && (false == _is_in_obs_bk_lines(_central_point)) ) {
            int cp_x = rand()*_sample_width_scale + _map_width /2;
            int cp_y = rand()*_sample_height_scale + _map_height/2;
            _central_point = Point2D(cp_x, cp_y);
        }
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
            p_obstacle->m_alpha_seg = Segment2D( p_obstacle->m_bk, *p_a_pt );
        }
        if ( p_b_pt ) {
            p_obstacle->m_beta_seg = Segment2D( p_obstacle->m_bk, *p_b_pt );
        }
    }

}

bool WorldMap::init_segments() {

    for( std::vector<Obstacle*>::iterator it=_obstacles.begin(); it!=_obstacles.end(); it++) {
        Obstacle* p_obstacle = (*it);
        for( std::vector<Obstacle*>::iterator itr=_obstacles.begin(); itr!=_obstacles.end(); itr++) {
            Obstacle* p_ref_obstacle = (*itr);
            // check alpha_seg with obstacles
            std::vector<Point2D> result = _intersect(p_obstacle->m_alpha_seg, p_ref_obstacle->m_segments);
        }
    }
}



bool WorldMap::_is_in_obstacle(Point2D point) {

    if( _pp_obstalce_info ) {
        int x = (int)point.x();
        int y = (int)point.y();
        if( _pp_obstalce_info[x][y] < OBSTACLE_THRESHOLD ) {
            return true;
        }
    }
    return false;
}

bool WorldMap::_is_in_obs_bk_lines(Point2D point) {

    for( std::vector<Line2D>::iterator it = _obs_bk_pair_lines.begin(); it != _obs_bk_pair_lines.end(); it++ ) {
        Line2D bk_line = (*it);
        if ( bk_line.has_on(_central_point)==true ) {
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

bool WorldMap::is_in_obstacle( Point2D point ) {
    for( std::vector<Obstacle*>::iterator it = _obstacles.begin(); it != _obstacles.end(); it++ ) {
        Obstacle * p_obstacle = (*it);
        if( p_obstacle->m_pgn.has_on_boundary( point ) == true ) {
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
            points.push_back(p);
        }
    }
    return points;
}
