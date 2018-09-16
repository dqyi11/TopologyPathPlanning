#include "topologyPathPlanning/homotopy/Region.hpp"
#include "topologyPathPlanning/homotopy/CgalUtil.hpp"

namespace topologyPathPlanning {

namespace homotopy {

SubRegion::SubRegion( Polygon2D poly , SubRegionSet* p_parent ) {

  mPoints.clear();
  mPolygon = Polygon2D();
  mpParent = p_parent;

  for( Polygon2D::Vertex_iterator it = poly.vertices_begin();
       it != poly.vertices_end(); it++ ) {
    Point2D p = *it;
    mPoints.push_back(p);
    mPolygon.push_back(p);
  }
  if (mPolygon.orientation() == CGAL::CLOCKWISE) {
    mPolygon.reverse_orientation();
  }
  mCentroid = getCentroid( mPolygon );
  mMinX = mPolygon.bbox().xmin();
  mMinY = mPolygon.bbox().ymin();
  mMaxX = mPolygon.bbox().xmax();
  mMaxY = mPolygon.bbox().ymax();
  mDistToCp = 0.0;
  mIndex = 0;
}

SubRegion::~SubRegion() {

  mPoints.clear();
  mPolygon.clear();
}

std::string SubRegion:: getName() {
  if( mpParent ) {
    std::stringstream ss;
    ss << mpParent->getName().c_str() << "-" << mIndex;
    return ss.str();
  }
  return "NA";
}

bool SubRegion::contains( Point2D point ) {
  if ( CGAL::ON_UNBOUNDED_SIDE != mPolygon.bounded_side( point ) ) { 
    return true;
  }
  return false;
}

Point2D SubRegion::samplePosition() {
  bool found = false;
  while (found == false) {
    float x_ratio = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
    float y_ratio = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
    int rnd_x = static_cast<int>(x_ratio*(mMaxX - mMinX)) + mMinX;
    int rnd_y = static_cast<int>(y_ratio*(mMaxY - mMinY)) + mMinY;

    Point2D rnd_point(rnd_x, rnd_y);
    if ( CGAL::ON_BOUNDED_SIDE == mPolygon.bounded_side( rnd_point ) ) {
      return rnd_point;
    }
  }
  return mCentroid;
}
  
SubRegionSet::SubRegionSet(std::list<Point2D> points, unsigned int idx) {

  mBoundaryPoints.clear();
  mIndex = idx;
  mSubregions.clear();
  mPolygon = Polygon2D();

  for( std::list<Point2D>::iterator it=points.begin(); it != points.end(); it++ ) {
    Point2D p = *it;
    mBoundaryPoints.push_back(p);
    mPolygon.push_back(p);
  }

  if(mPolygon.orientation() == CGAL::CLOCKWISE) {
    mPolygon.reverse_orientation();
  }
  mCentroid = getCentroid( mPolygon );

}

SubRegionSet::~SubRegionSet() {

  mBoundaryPoints.clear();
}

std::string SubRegionSet:: getName() {
  std::stringstream ss;
  ss << "R" << mIndex;
  return ss.str();
}


} // homotopy

} // topologyPathPlanning
