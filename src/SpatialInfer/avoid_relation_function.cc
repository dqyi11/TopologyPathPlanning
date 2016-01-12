#include "avoid_relation_function.h"

using namespace std;
using namespace homotopy;
using namespace topology_inference;

AvoidRelationFunction::AvoidRelationFunction() {
  mp_obstacle = NULL;
}

AvoidRelationFunction::~AvoidRelationFunction() {
  mp_obstacle = NULL;
}

vector< pair<ReferenceFrame*, bool> > AvoidRelationFunction::get_rules( ReferenceFrameSet* p_reference_frame_set ) {
  vector< pair<ReferenceFrame*, bool> > rules;
  for( vector<ReferenceFrame*>::iterator it = p_reference_frame_set->get_reference_frames().begin();
       it != p_reference_frame_set->get_reference_frames().end(); it++ ) {
    ReferenceFrame* p_rf = (*it);
    if( p_rf ) {
      if( p_rf->mp_line_subsegment ) {
        if( p_rf->mp_line_subsegment->is_connected( mp_obstacle ) ) {
          rules.push_back( make_pair( p_rf, false ) );
        }
      }
    }
  }  
  return rules;  
}

string AvoidRelationFunction::get_name() {
  string name = "AVOID";
  if( mp_obstacle ){
    name += "(" + mp_obstacle->get_name() + ")";
  }
  return name;
}
