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
  
  return rules;  
}

string AvoidRelationFunction::get_name() {
  string name = "AVOID";
  if( mp_obstacle ){
    name += "(" + mp_obstacle->get_name() + ")";
  }
  return name;
}
