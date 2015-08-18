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
    SubRegionSet(Segment2D* p_ray1, Segment2D* p_ray2, unsigned int idx);
    ~SubRegionSet();

    Segment2D* mp_ray1;
    Segment2D* mp_ray2;
    unsigned int m_index;

    std::vector<SubRegion*> m_subregions;
};

#endif // REGION_H
