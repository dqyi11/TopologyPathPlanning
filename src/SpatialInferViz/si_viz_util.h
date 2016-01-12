#ifndef SI_VIZ_UTIL_H_
#define SI_VIZ_UTIL_H_

#include <CGAL/squared_distance_2.h>
#include <QPoint>

namespace topology_inference {

  inline QPoint toQPoint( Point2D point ) {
    double x = CGAL::to_double( point.x() );
    double y = CGAL::to_double( point.y() );
    return QPoint((int)x, (int)y ); 
 }

  /* 
  inline QPoint toQPoint( POS2D pos ) {
    return QPoint(pos[0], pos[1]);
  }*/

  inline Point2D toPoint2D( QPoint pos ) {
    return Point2D( pos.x(), pos.y() );
  }

}

#endif /* ML_VIZ_UTIL_H_ */
