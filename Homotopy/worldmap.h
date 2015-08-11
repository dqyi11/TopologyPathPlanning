#ifndef WORLDMAP_H
#define WORLDMAP_H

#include "obstacle.h"


class WorldMap {
public:
    WorldMap( int width, int height );

    bool load_obstalce_info( int** pp_obstacle_info );

    bool init();

private:
    int _map_width;
    int _map_height;

    int** _pp_obstalce_info;

    std::vector<Obstacle*> _obstacles;
    std::vector<Line2D>    _obs_bk_pair_lines;
    Point2D                _central_point;
};

#endif // WORLDMAP_H
