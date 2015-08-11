#ifndef WORLDMAP_H
#define WORLDMAP_H

#include "obstacle.h"


class WorldMap {
public:
    WorldMap( int width, int height );

    bool load_obstalce_info( int** pp_obstacle_info );
    bool _is_in_obstacle(Point2D point);
    bool _is_in_obs_bk_lines(Point2D point);
    bool init();

    Point2D _find_intersection_with_boundary(Ray2D ray);

private:
    int _map_width;
    int _map_height;

    int** _pp_obstalce_info;

    std::vector<Obstacle*> _obstacles;
    std::vector<Line2D>    _obs_bk_pair_lines;
    std::vector<Segment2D> _boundary_lines;
    Point2D                _central_point;
    Line2D                 _x_min_line;
    Line2D                 _x_max_line;
    Line2D                 _y_min_line;
    Line2D                 _y_max_line;
};

#endif // WORLDMAP_H
