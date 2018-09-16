#ifndef TOPOLOGYPATHPLANNING_TARRTS_MLUTIL_HPP
#define TOPOLOGYPATHPLANNING_TARRTS_MLUTIL_HPP

#include <CGAL/squared_distance_2.h>

#include "topologyPathPlanning/homotopy/WorldDatatype.hpp"

namespace topologyPathPlanning {

namespace tarrts {

  inline homotopy::Point2D toPoint2D( POS2D pos ) {
    return homotopy::Point2D( pos[0], pos[1] );
  }

  inline POS2D toPOS2D( homotopy::Point2D point ) {
    double x = CGAL::to_double( point.x() );
    double y = CGAL::to_double( point.y() );
    return POS2D( x, y ); 
  }

} // tarrts

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TARRTS_MLUTIL_HPP
