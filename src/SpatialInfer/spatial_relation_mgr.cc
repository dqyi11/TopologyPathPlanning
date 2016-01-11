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

vector< string > SpatialRelationMgr::get_spatial_relation_function_names() {
  vector< string > names;
  for(unsigned int i=0; i < mp_functions.size(); i++) {
    SpatialRelationFunction* p_func = mp_functions[i];
    if( p_func ) {
      names.push_back( p_func->get_name() );
    }
  }
  return names;
}

bool SpatialRelationMgr::has_spatial_relation_function( string name ) {
  for(unsigned int i=0; i < mp_functions.size(); i++) {
    SpatialRelationFunction* p_func = mp_functions[i];
    if( p_func ) {
      if( p_func->get_name() == name ) {
        return true;
      }
    }
  }
  return false;
}

void SpatialRelationMgr::remove_spatial_relation_function( string name ) {

  for( vector<SpatialRelationFunction*>::iterator it = mp_functions.begin();
       it != mp_functions.end(); /* it ++ */ ) { 
    SpatialRelationFunction* p_func = (*it);
    if( p_func && ( p_func->get_name() == name ) ) {
      mp_functions.erase( it );
    }
    else {
      ++ it;
    }
  }
}
