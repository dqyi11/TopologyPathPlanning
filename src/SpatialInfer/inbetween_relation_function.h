#ifndef INBETWEEN_RELATION_FUNC_H_
#define INBETWEEN_RELATION_FUNC_H_

#include "obstacle.h"
#include "spatial_relation_function.h"

namespace topology_inference {

  class InBetweenRelationFunction : public SpatialRelationFunction {
  public:
    InBetweenRelationFunction();
    virtual ~InBetweenRelationFunction();

    virtual std::vector< std::pair< homotopy::ReferenceFrame*, bool> > get_reference_frames( homotopy::ReferenceFrameSet* p_reference_frame_set ); 
    virtual std::string get_name();

    std::vector<homotopy::Obstacle*> mp_obstacles;
  };
}

#endif // INBETWEEN_RELATION_FUNC_H_
