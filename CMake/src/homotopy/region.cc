#include "region.h"

SubRegion::SubRegion(std::vector<Point2D> points) {

    m_points.clear();
    for( std::vector<Point2D>::iterator it=points.begin(); it!=points.end(); it++ ) {
        Point2D p = (*it);
        m_points.push_back(p);
    }

}

SubRegionSet::SubRegionSet(std::vector<Point2D> points, unsigned int idx) {

    m_boundary_points.clear();
    m_index = idx;
    m_subregions.clear();

    for( unsigned int i=0; i < points.size(); i++ ) {
        m_boundary_points.push_back((points[i]));
    }
}

