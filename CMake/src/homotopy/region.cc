#include "region.h"

SubRegion::SubRegion( Polygon2D poly ) {

    m_points.clear();
    m_polygon = Polygon2D();

    for( Polygon2D::Vertex_iterator it = poly.vertices_begin();
         it != poly.vertices_end(); it++ ) {
        Point2D p = *it;
        m_points.push_back(p);
        m_polygon.push_back(p);
    }

}

SubRegionSet::SubRegionSet(std::vector<Point2D> points, unsigned int idx) {

    m_boundary_points.clear();
    m_index = idx;
    m_subregions.clear();
    m_polygon = Polygon2D();

    for( unsigned int i=0; i < points.size(); i++ ) {
        m_boundary_points.push_back((points[i]));
        m_polygon.push_back(points[i]);
    }
}

SubRegionSet::~SubRegionSet() {

    m_boundary_points.clear();
}
