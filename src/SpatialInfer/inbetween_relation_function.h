#ifndef INBETWEEN_RELATION_FUNC_H_
#define INBETWEEN_RELATION_FUNC_H_

#include "obstacle.h"
#include "spatial_relation_function.h"

namespace topology_inference {

  class InBetweenRelationFunction : SpatialRelationFunction {
  public:
    InBetweenRelationFunction();
    virtual ~InBetweenRelationFunction();

    virtual std::vector<homotopy::ReferenceFrame*> get_reference_frames( homotopy::ReferenceFrameSet* p_reference_frame_set ); 

    std::vector<homotopy::Obstacle*> m_obstacles;
  };
}

#endif // INBETWEEN_RELATION_FUNC_H_
