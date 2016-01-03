#ifndef SPATIAL_RELATION_MGR_H
#define SPATIAL_RELATION_MGR_H

#include <vector>
#include "spatial_relation_function.h"

namespace topology_inference {

  class SpatialRelationMgr {
  public:
    SpatialRelationMgr();
    virtual ~SpatialRelationMgr();

    std::vector<homotopy::ReferenceFrame*> get_reference_frames( homotopy::ReferenceFrameSet* p_reference_frame_set );

    std::vector<SpatialRelationFunction*> mp_functions; 
  };
}

#endif // SPATIAL_RELATION_MGR_H
