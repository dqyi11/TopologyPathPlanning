#ifndef WORLDMAP_H
#define WORLDMAP_H

#include <libxml/tree.h>
#include "obstacle.h"
#include "region.h"

namespace homotopy {

  class WorldMap {
  public:
    WorldMap();
    WorldMap( int width, int height );
    virtual ~WorldMap();

    bool resize( int width, int height );
    bool load_obstacle_info( std::vector< std::vector<Point2D> > polygons);
    bool init( bool rand_init_points = true );

    bool _is_in_obstacle( Point2D point );
    bool _is_in_obs_bk_lines( Point2D point );

    virtual void to_xml( const std::string& filename )const;
    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( const std::string& filename );
    virtual void from_xml( xmlNodePtr root );

    std::vector<Point2D> _intersect( Segment2D seg, std::vector<Segment2D> segments );
    Point2D* _find_intersection_with_boundary( Ray2D* p_ray );

    int get_width() const { return _map_width; }
    int get_height() const { return _map_height; }

    SubRegion* find_subregion( std::string name );
    LineSubSegment* find_linesubsegment( std::string name );

    Point2D get_central_point() const { return _central_point; }
    std::vector<Obstacle*> get_obstacles() const { return _obstacles; }
    std::vector<SubRegionSet*> get_subregion_set() const { return _region_sets; }
    std::vector<SubRegion*> get_subregions() const { return _subregions; } 
    std::vector<LineSubSegmentSet*> get_linesubsegment_set() const { return _line_segments; }

    double get_distance_to_central_point( Point2D point );

    SubRegion * in_subregion( Point2D point );
  protected:
    bool _init_points();
    bool _init_rays();
    bool _init_segments();
    bool _init_regions();

    std::list<Point2D>       _intersect_with_boundaries( LineSubSegmentSet* p_segment1, LineSubSegmentSet* p_segment2 );
    std::vector<SubRegion*>  _get_subregions( SubRegionSet* p_region );
    bool                     _is_intersected( Polygon2D poly, Segment2D seg, double delta );

  private:
    int _map_width;
    int _map_height;

    int _sample_width_scale;
    int _sample_height_scale;

    std::vector<Obstacle*>          _obstacles;
    std::vector<LineSubSegmentSet*> _line_segments;
    std::vector<SubRegionSet*>      _region_sets;
    std::vector<SubRegion*>         _subregions;

    std::vector<Line2D>             _obs_bk_pair_lines;
    std::vector<Segment2D>          _boundary_lines;
    std::vector<Segment2D>          _center_corner_lines;

    Point2D                _central_point;
    Segment2D              _x_min_line;
    Segment2D              _x_max_line;
    Segment2D              _y_min_line;
    Segment2D              _y_max_line;
  };

  std::ostream& operator<<( std::ostream& out, const WorldMap& other );

}

#endif // WORLDMAP_H
