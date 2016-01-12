#include "inbetween_relation_function.h"

using namespace std;
using namespace homotopy;
using namespace topology_inference;

InBetweenRelationFunction::InBetweenRelationFunction() {
  mp_obstacles.clear();
}

InBetweenRelationFunction::~InBetweenRelationFunction() {
  mp_obstacles.clear();
}

vector< pair<ReferenceFrame*, bool> > InBetweenRelationFunction::get_rules( ReferenceFrameSet* p_reference_frame_set ) {
  vector< pair<ReferenceFrame*, bool> > rules;

  if( p_reference_frame_set ) {
    for( unsigned int i=0; i<p_reference_frame_set->get_reference_frames().size(); i++ ) {
      ReferenceFrame* p_ref = p_reference_frame_set->get_reference_frames()[i];
      if( p_ref ) {
        
      }
    }
  }
  return rules;  
}

string InBetweenRelationFunction::get_name() {
  string name = "IN_BETWEEN(";
  for(unsigned int i=0; i<mp_obstacles.size(); i++) {
    if( i < mp_obstacles.size()-1 ) {
      name += mp_obstacles[i]->get_name() + ",";
    }
    else{
      name += mp_obstacles[i]->get_name();
    }
  }  
  name += ")";
  return name;
}
