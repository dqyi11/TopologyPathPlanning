#ifndef AVOID_RELATION_FUNC_H_
#define AVOID_RELATION_FUNC_H_

#include "obstacle.h"
#include "spatial_relation_function.h"

namespace topology_inference {

  class AvoidRelationFunction : public SpatialRelationFunction {
  public:
    AvoidRelationFunction();
    virtual ~AvoidRelationFunction();

    virtual std::vector< std::pair<homotopy::ReferenceFrame*, bool> > get_reference_frames( homotopy::ReferenceFrameSet* p_reference_frame_set ); 
    virtual std::string get_name();

    homotopy::Obstacle* mp_obstacle;
  };
}

#endif // AVOID_RELATION_FUNC_H_
