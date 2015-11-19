#ifndef CGAL_UTIL_H_
#define CGAL_UTIL_H_

#include <CGAL/centroid.h>
#include "world_datatype.h"

namespace homotopy {

  inline Point2D get_centroid(Polygon2D poly) {
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

}

#endif /* CGAL_UTIL_H_ */
