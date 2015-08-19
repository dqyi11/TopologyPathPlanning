#ifndef REGION_H
#define REGION_H

#include <vector>
#include "world_datatype.h"


class SubRegion {
public:
    SubRegion( Polygon2D poly );
    virtual ~SubRegion();

    std::vector<Point2D> m_points;
    Polygon2D            m_polygon;
};


class SubRegionSet {
public:
    SubRegionSet(std::vector<Point2D> points, unsigned int idx);
    virtual ~SubRegionSet();

    Polygon2D    m_polygon;
    unsigned int m_index;    
    std::vector<Point2D>    m_boundary_points;
    std::vector<SubRegion*> m_subregions;    
};

#endif // REGION_H
