#include "topologyPathPlanning/spatialinfer/AvoidRelationFunction.hpp"

using namespace std;
using namespace topologyPathPlanning::homotopy;

namespace topologyPathPlanning {

namespace topologyinference {

AvoidRelationFunction::AvoidRelationFunction() {
  mp_obstacle = NULL;
}

AvoidRelationFunction::~AvoidRelationFunction() {
  mp_obstacle = NULL;
}

vector< pair<ReferenceFrame*, bool> > AvoidRelationFunction::get_rules( ReferenceFrameSet* p_reference_frame_set ) {
  vector< pair<ReferenceFrame*, bool> > rules;
  for( vector<ReferenceFrame*>::iterator it = p_reference_frame_set->getReferenceFrames().begin();
       it != p_reference_frame_set->getReferenceFrames().end(); it++ ) {
    ReferenceFrame* p_rf = (*it);
    if( p_rf ) {
      if( p_rf->mpLineSubsegment ) {
        if( p_rf->mpLineSubsegment->isConnected( mp_obstacle ) ) {
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
    name += "(" + mp_obstacle->getName() + ")";
  }
  return name;
}

} // topologyinference

} // topologyPathPlanning
