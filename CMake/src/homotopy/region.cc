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

    if (m_polygon.orientation() == CGAL::CLOCKWISE) {
        m_polygon.reverse_orientation();
    }
}

SubRegion::~SubRegion() {

    m_points.clear();
    m_polygon.clear();
}

SubRegionSet::SubRegionSet(std::list<Point2D> points, unsigned int idx) {

    m_boundary_points.clear();
    m_index = idx;
    m_subregions.clear();
    m_polygon = Polygon2D();

    for( std::list<Point2D>::iterator it=points.begin(); it != points.end(); it++ ) {
        Point2D p = *it;
        m_boundary_points.push_back(p);
        m_polygon.push_back(p);
    }

    if(m_polygon.orientation() == CGAL::CLOCKWISE) {
        m_polygon.reverse_orientation();
    }
}

SubRegionSet::~SubRegionSet() {

    m_boundary_points.clear();
}
