#ifndef ML_UTIL_H_
#define ML_UTIL_H_

#include <CGAL/squared_distance_2.h>

namespace mlrrts {

  inline Point2D toPoint2D( POS2D pos ) {
    return Point2D( pos[0], pos[1] );
  }

  inline POS2D toPOS2D( Point2D point ) {
    double x = CGAL::to_double( point.x() );
    double y = CGAL::to_double( point.y() );
    return POS2D( x, y ); 
  }

}

#endif /* ML_UTIL_H_ */
