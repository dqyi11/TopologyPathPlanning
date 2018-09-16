#include "topologyPathPlanning/spatialinfer/AvoidRelationFunction.hpp"

using namespace std;
using namespace topologyPathPlanning::homotopy;

namespace topologyPathPlanning {

namespace topologyinference {

AvoidRelationFunction::AvoidRelationFunction() {
  mpObstacle = NULL;
}

AvoidRelationFunction::~AvoidRelationFunction() {
  mpObstacle = NULL;
}

vector< pair<ReferenceFrame*, bool> > AvoidRelationFunction::getRules( ReferenceFrameSet* p_reference_frame_set ) {
  vector< pair<ReferenceFrame*, bool> > rules;
  for( vector<ReferenceFrame*>::iterator it = p_reference_frame_set->getReferenceFrames().begin();
       it != p_reference_frame_set->getReferenceFrames().end(); it++ ) {
    ReferenceFrame* p_rf = (*it);
    if( p_rf ) {
      if( p_rf->mpLineSubsegment ) {
        if( p_rf->mpLineSubsegment->isConnected( mpObstacle ) ) {
          rules.push_back( make_pair( p_rf, false ) );
        }
      }
    }
  }  
  return rules;  
}

string AvoidRelationFunction::getName() {
  string name = "AVOID";
  if( mpObstacle ){
    name += "(" + mpObstacle->getName() + ")";
  }
  return name;
}

} // topologyinference

} // topologyPathPlanning
