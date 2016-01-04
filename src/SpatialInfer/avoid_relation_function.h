#ifndef INBETWEEN_RELATION_FUNC_H_
#define INBETWEEN_RELATION_FUNC_H_

#include "obstacle.h"
#include "spatial_relation_function.h"

namespace topology_inference {

  class AvoidRelationFunction : SpatialRelationFunction {
  public:
    AvoidRelationFunction();
    virtual ~AvoidRelationFunction();

    virtual std::vector< std::pair<homotopy::ReferenceFrame*, bool> > get_reference_frames( homotopy::ReferenceFrameSet* p_reference_frame_set ); 

    homotopy::Obstacle* mp_obstacle;
  };
}

#endif // INBETWEEN_RELATION_FUNC_H_
