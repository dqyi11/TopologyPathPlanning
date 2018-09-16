#ifndef CGAL_UTIL_H_
#define CGAL_UTIL_H_

#include <math.h>
#include <CGAL/centroid.h>
#include "topologyPathPlanning/homotopy/WorldDatatype.hpp"

namespace topologyPathPlanning {

namespace homotopy {

  inline Point2D getCentroid(Polygon2D poly) {
    std::list<Point2D> verList;
    Vertex_circulator e = poly.vertices_circulator();
    Vertex_circulator end;
    if( e != NULL ) {
      end = e;
      do {
        verList.push_back( *e );
        ++e;
      }
      while( e!= end);
    }
    Point2D c2 = CGAL::centroid( verList.begin(), verList.end() );
    return c2; 
  }
  
  inline double getRadius(Point2D from, Point2D to) {
    double from_x = CGAL::to_double( from.x() );    
    double from_y = CGAL::to_double( from.y() );    
    double to_x = CGAL::to_double( to.x() );    
    double to_y = CGAL::to_double( to.y() );    
   
    return atan2( to_y - from_y, to_x - from_x );

  }

} // homotopy

} // topologyPathPlanning

#endif /* CGAL_UTIL_H_ */
