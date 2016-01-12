#ifndef SPATIAL_RELATION_FUNC_H_
#define SPATIAL_RELATION_FUNC_H_

#include <utility>
#include <vector>
#include <string>
#include "reference_frames.h"

namespace topology_inference {

  class SpatialRelationFunction {
  public:
    SpatialRelationFunction() {};
    virtual ~SpatialRelationFunction() {};

    virtual std::vector< std::pair<homotopy::ReferenceFrame*, bool> > get_rules( homotopy::ReferenceFrameSet* p_reference_frame_set ) = 0; 
    virtual std::string get_name() = 0;
  };
}

#endif // SPATIAL_RELATION_FUNC_H_
