#include "spatial_relation_mgr.h"

using namespace std;
using namespace homotopy;
using namespace topology_inference;

SpatialRelationMgr::SpatialRelationMgr( WorldMap* worldmap ) {
  mp_worldmap = worldmap;
  mp_functions.clear();
  m_start_x = -1;
  m_start_y = -1;
  m_goal_x = -1;
  m_goal_y = -1;
}

SpatialRelationMgr::~SpatialRelationMgr() {
  mp_worldmap = NULL;
  mp_functions.clear();
}

vector< pair<ReferenceFrame*, bool> > SpatialRelationMgr::get_reference_frames( ReferenceFrameSet* p_reference_frame_set ) {
  vector< pair<ReferenceFrame*, bool> > reference_frames;
  for( unsigned int i=0; i < mp_functions.size(); i++ ) {
    vector< pair<ReferenceFrame*, bool> > rfs = mp_functions[i]->get_reference_frames( p_reference_frame_set );
    for( unsigned int j=0; j < rfs.size(); j ++ ) {
      reference_frames.push_back( rfs[j] );
    } 
  }
  return reference_frames;
}
