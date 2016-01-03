#include "spatial_relation_mgr.h"

using namespace std;
using namespace homotopy;
using namespace topology_inference;

SpatialRelationMgr::SpatialRelationMgr() {
  mp_functions.clear();
}

SpatialRelationMgr::~SpatialRelationMgr() {
  mp_functions.clear();
}

vector<ReferenceFrame*> SpatialRelationMgr::get_reference_frames( ReferenceFrameSet* p_reference_frame_set ) {
  vector<ReferenceFrame*> reference_frames;
  for( unsigned int i=0; i < mp_functions.size(); i++ ) {
    vector<ReferenceFrame*> rfs = mp_functions[i]->get_reference_frames( p_reference_frame_set );
    for( unsigned int j=0; j < rfs.size(); j ++ ) {
      reference_frames.push_back( rfs[j] );
    } 
  }
  return reference_frames;
}
