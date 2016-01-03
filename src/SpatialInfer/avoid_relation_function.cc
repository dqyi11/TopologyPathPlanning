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

vector<ReferenceFrame*> AvoidRelationFunction::get_reference_frames( ReferenceFrameSet* p_reference_frame_set ) {
  vector<ReferenceFrame*> reference_frames;

  return reference_frames;  
}
