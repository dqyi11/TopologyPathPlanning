#ifndef SPATIAL_RELATION_MGR_H
#define SPATIAL_RELATION_MGR_H

#include <vector>
#include "worldmap.h"
#include "spatial_relation_function.h"

namespace topology_inference {

  class SpatialRelationMgr {
  public:
    SpatialRelationMgr(homotopy::WorldMap* map);
    virtual ~SpatialRelationMgr();

    std::vector< std::pair<homotopy::ReferenceFrame*, bool> > get_reference_frames( homotopy::ReferenceFrameSet* p_reference_frame_set );
    std::vector< std::string > get_spatial_relation_function_names();
  

    std::vector<SpatialRelationFunction*> mp_functions; 
    homotopy::WorldMap*                   mp_worldmap;
    int m_start_x;
    int m_start_y;
    int m_goal_x;
    int m_goal_y;
  };
}

#endif // SPATIAL_RELATION_MGR_H
