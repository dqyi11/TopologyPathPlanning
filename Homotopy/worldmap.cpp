#include "opencv2/core/core.hpp"

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

    }
}

