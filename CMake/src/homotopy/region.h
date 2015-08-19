#ifndef REGION_H
#define REGION_H

#include <vector>
#include "world_datatype.h"


class SubRegion {
public:
    SubRegion( std::vector<Point2D> points);
    ~SubRegion();

    std::vector<Point2D> m_points;
};


class SubRegionSet {
public:
    SubRegionSet(std::vector<Point2D> points, unsigned int idx);
    ~SubRegionSet();

    unsigned int m_index;    
    std::vector<Point2D>    m_boundary_points;
    std::vector<SubRegion*> m_subregions;    
};

#endif // REGION_H
