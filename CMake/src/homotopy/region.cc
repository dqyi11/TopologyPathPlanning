#include "region.h"

SubRegion::SubRegion(std::vector<Point2D> points) {

    m_points.clear();
    for( std::vector<Point2D>::iterator it=points.begin(); it!=points.end(); it++ ) {
        Point2D p = (*it);
        m_points.push_back(p);
    }
}

SubRegionSet::SubRegionSet(Segment2D* p_ray1, Segment2D* p_ray2, unsigned int idx) {

    mp_ray1 = p_ray1;
    mp_ray2 = p_ray2;
    m_index = idx;

    m_subregions.clear();
}

