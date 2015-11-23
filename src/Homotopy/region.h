#ifndef REGION_H
#define REGION_H

#include <vector>
#include "world_datatype.h"
#include "line_subsegment.h"

namespace homotopy {

  class SubRegionSet;

  class SubRegion {
  public:
    SubRegion( Polygon2D poly , SubRegionSet* p_parent );
    virtual ~SubRegion();
    
    bool contains( Point2D point );
    Point2D sample_position();  

 
    std::string get_name();

    std::vector<Point2D>         m_points;
    Polygon2D                    m_polygon;
    Point2D                      m_centroid;
    double                       m_dist_to_cp;
    unsigned int                 m_index;
    std::vector<LineSubSegment*> m_neighbors;
    bool                         m_is_connected_to_central_point;
  protected:
    SubRegionSet*                mp_parent;
    int                          _min_x;
    int                          _min_y;
    int                          _max_x;
    int                          _max_y;
    
  };

  class SubRegionSet {
  public:
    SubRegionSet(std::list<Point2D> points, unsigned int idx);
    virtual ~SubRegionSet();

    std::string get_name();

    Polygon2D    m_polygon;
    unsigned int m_index;    
    std::vector<Point2D>    m_boundary_points;
    std::vector<SubRegion*> m_subregions;
    Point2D                 m_centroid;

    LineSubSegmentSet*      mp_line_segments_a;
    LineSubSegmentSet*      mp_line_segments_b;
  };

}

#endif // REGION_H
