#include "topologyPathPlanning/homotopy/Region.hpp"
#include "topologyPathPlanning/homotopy/CgalUtil.hpp"

namespace topologyPathPlanning {

namespace homotopy {

SubRegion::SubRegion( Polygon2D poly , SubRegionSet* p_parent ) {

  m_points.clear();
  m_polygon = Polygon2D();
  mp_parent = p_parent;

  for( Polygon2D::Vertex_iterator it = poly.vertices_begin();
       it != poly.vertices_end(); it++ ) {
    Point2D p = *it;
    m_points.push_back(p);
    m_polygon.push_back(p);
  }
  if (m_polygon.orientation() == CGAL::CLOCKWISE) {
    m_polygon.reverse_orientation();
  }
  m_centroid = get_centroid( m_polygon );
  _min_x = m_polygon.bbox().xmin();
  _min_y = m_polygon.bbox().ymin();
  _max_x = m_polygon.bbox().xmax();
  _max_y = m_polygon.bbox().ymax();
  m_dist_to_cp = 0.0;
  m_index = 0;
}

SubRegion::~SubRegion() {

  m_points.clear();
  m_polygon.clear();
}

std::string SubRegion:: get_name() {
  if( mp_parent ) {
    std::stringstream ss;
    ss << mp_parent->get_name().c_str() << "-" << m_index;
    return ss.str();
  }
  return "NA";
}

bool SubRegion::contains( Point2D point ) {
  if ( CGAL::ON_UNBOUNDED_SIDE != m_polygon.bounded_side( point ) ) { 
    return true;
  }
  return false;
}

Point2D SubRegion::sample_position() {
  bool found = false;
  while (found == false) {
    float x_ratio = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
    float y_ratio = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
    int rnd_x = static_cast<int>(x_ratio*(_max_x - _min_x)) + _min_x;
    int rnd_y = static_cast<int>(y_ratio*(_max_y - _min_y)) + _min_y;

    Point2D rnd_point(rnd_x, rnd_y);
    if ( CGAL::ON_BOUNDED_SIDE == m_polygon.bounded_side( rnd_point ) ) {
      return rnd_point;
    }
  }
  return m_centroid;
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
  m_centroid = get_centroid( m_polygon );

}

SubRegionSet::~SubRegionSet() {

  m_boundary_points.clear();
}

std::string SubRegionSet:: get_name() {
  std::stringstream ss;
  ss << "R" << m_index;
  return ss.str();
}


} // homotopy

} // topologyPathPlanning
