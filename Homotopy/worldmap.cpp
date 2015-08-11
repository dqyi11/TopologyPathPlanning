#include "opencv2/core/core.hpp"
#include <CGAL/intersections.h>
#include "worldmap.h"

#define NO_OBSTACLE_PIX    255
#define OBSTACLE_PIX       0
#define OBSTACLE_THRESHOLD 200

WorldMap::WorldMap( int width, int height ) {
    _map_width = width;
    _map_height = height;
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

bool WorldMap::load_obstalce_info( int** pp_obstacle_info ) {
    for( int i=0; i<_map_width; i++ ) {
        _pp_obstalce_info[i] = new int[_map_height];
        for ( int j=0; j<_map_height; i++ ) {
            if ( pp_obstacle_info[i][j] > OBSTACLE_THRESHOLD ) {
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
        p_obstacle->m_beta_ray = Ray2D( p_obstacle->m_bk, Point2D(2*p_obstacle->m_bk.x-_central_point.x, 2*p_obstacle->m_bk.y-_central_point.y) );

        p_obstacle->m_alpha_ray;
    }

}

bool WorldMap::_is_in_obstacle(Point2D point) {

    if( _pp_obstalce_info ) {
        int x = (int)point.x;
        int y = (int)point.y;
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
        cpp11::result_of<K::Intersect_2(Segment2D, Ray2D)>::type result = intersection(seg, ray);
        if (result) {
            Point2D* p = boost::get<Point2D>(&*result);
            return p;
        }
    }
    return NULL;
}
