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

vector<ReferenceFrame*> InBetweenRelationFunction::get_reference_frames( ReferenceFrameSet* p_reference_frame_set ) {
  vector<ReferenceFrame*> reference_frames;

  return reference_frames;  
}
