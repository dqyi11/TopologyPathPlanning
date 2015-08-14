#ifndef WORLDMAP_H
#define WORLDMAP_H

#include "obstacle.h"


class WorldMap {
public:
    WorldMap( int width, int height );

    bool load_obstacle_info( std::vector< std::vector<Point2D> > polygons);
    bool init();
    bool init_segments();

    bool _is_in_obstacle( Point2D point );
    bool _is_in_obs_bk_lines( Point2D point );

    std::vector<Point2D> _intersect( Segment2D seg, std::vector<Segment2D> segments );
    Point2D* _find_intersection_with_boundary( Ray2D ray );

    int get_width() { return _map_width; }
    int get_height() { return _map_height; }

    Point2D& get_central_point() { return _central_point; }
    std::vector<Obstacle*>& get_obstacles() { return _obstacles; }

private:
    int _map_width;
    int _map_height;

    int _sample_width_scale;
    int _sample_height_scale;

    std::vector<Obstacle*> _obstacles;
    std::vector<Line2D>    _obs_bk_pair_lines;
    std::vector<Segment2D> _boundary_lines;
    Point2D                _central_point;
    Segment2D              _x_min_line;
    Segment2D              _x_max_line;
    Segment2D              _y_min_line;
    Segment2D              _y_max_line;
};

#endif // WORLDMAP_H
