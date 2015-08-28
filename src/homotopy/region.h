#ifndef REGION_H
#define REGION_H

#include <vector>
#include "world_datatype.h"
#include "line_subsegment.h"

class SubRegionSet;

class SubRegion {
public:
    SubRegion( Polygon2D poly , SubRegionSet* p_parent );
    virtual ~SubRegion();

    std::string get_name();

    std::vector<Point2D> m_points;
    Polygon2D            m_polygon;
    Point2D              m_centroid;
    double               m_dist_to_cp;
    unsigned int         m_index;
    SubRegionSet*        mp_parent;

    std::vector<LineSubSegment*> m_neighbors;
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

    LineSubSegmentSet* mp_line_segments_a;
    LineSubSegmentSet* mp_line_segments_b;
};

#endif // REGION_H
