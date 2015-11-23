#ifndef ML_UTIL_H_
#define ML_UTIL_H_

#include <CGAL/squared_distance_2.h>
#include <QPoint>

namespace mlrrts {

  inline QPoint toQPoint( Point2D point ) {
    double x = CGAL::to_double( point.x() );
    double y = CGAL::to_double( point.y() );
    return QPoint((int)x, (int)y ); 
  }

  inline QPoint toQPoint( POS2D pos ) {
    return QPoint(pos[0], pos[1]);
  }
}

#endif /* ML_UTIL_H_ */
