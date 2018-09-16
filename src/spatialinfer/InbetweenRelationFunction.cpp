#include "topologyPathPlanning/spatialinfer/InbetweenRelationFunction.hpp"

using namespace std;
using namespace topologyPathPlanning::homotopy;

namespace topologyPathPlanning {

namespace topologyinference {

InBetweenRelationFunction::InBetweenRelationFunction() {
  mpObstacles.clear();
}

InBetweenRelationFunction::~InBetweenRelationFunction() {
  mpObstacles.clear();
}

vector< pair<ReferenceFrame*, bool> > InBetweenRelationFunction::getRules( ReferenceFrameSet* p_reference_frame_set ) {
  vector< pair<ReferenceFrame*, bool> > rules;

  if( p_reference_frame_set ) {
    // check whether there is a reference frame connecting to reference frames
    for( unsigned int i=0; i<p_reference_frame_set->getReferenceFrames().size(); i++ ) {
      ReferenceFrame* p_ref = p_reference_frame_set->getReferenceFrames()[i];
      if( p_ref ) {
        if( p_ref->mpLineSubsegment ) {
          if( p_ref->mpLineSubsegment->isConnected( mpObstacles[0] ) &&
              p_ref->mpLineSubsegment->isConnected( mpObstacles[1] ) ) {
            rules.push_back( make_pair( p_ref, true ) );
          }
        }    
      }
    }
    if( rules.size() == 0 ) {
      for( unsigned int i=0; i<p_reference_frame_set->getReferenceFrames().size(); i++ ) {
        ReferenceFrame* p_ref = p_reference_frame_set->getReferenceFrames()[i];
        if( p_ref ) {
          if( p_ref->mpLineSubsegment ) {
            if( p_ref->mpLineSubsegment->isConnected( mpObstacles[0] ) &&
                p_ref->mpLineSubsegment->mIsConnectedToCentralPoint ) {

              rules.push_back( make_pair( p_ref, true ) );
            }
            else if( p_ref->mpLineSubsegment->isConnected( mpObstacles[1] ) &&
                     p_ref->mpLineSubsegment->mIsConnectedToCentralPoint ) {
              rules.push_back( make_pair( p_ref, true ) );
            }
          }
        }
      }
    }
  }
  return rules;  
}

string InBetweenRelationFunction::getName() {
  string name = "IN_BETWEEN(";
  for(unsigned int i=0; i<mpObstacles.size(); i++) {
    if( i < mpObstacles.size()-1 ) {
      name += mpObstacles[i]->getName() + ",";
    }
    else{
      name += mpObstacles[i]->getName();
    }
  }  
  name += ")";
  return name;
}

} // topologyinference

} // topologyPathPlanning
