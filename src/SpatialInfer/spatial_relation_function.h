#ifndef SPATIAL_RELATION_FUNC_H_
#define SPATIAL_RELATION_FUNC_H_

#include <vector>
#include "reference_frames.h"

namespace topology_inference {

  class SpatialRelationFunction {
  public:
    SpatialRelationFunction();
    virtual ~SpatialRelationFunction();

    virtual std::vector<homotopy::ReferenceFrame*> get_reference_frames( homotopy::ReferenceFrameSet* p_reference_frame_set ) = 0; 
  };
}

#endif // SPATIAL_RELATION_FUNC_H_
