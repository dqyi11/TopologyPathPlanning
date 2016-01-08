#include "sideof_relation_function.h"

using namespace std;
using namespace homotopy;
using namespace topology_inference;

SideOfRelationFunction::SideOfRelationFunction( side_type_t side_type ) {
  m_type = side_type;
  mp_obstacle = NULL;
}

SideOfRelationFunction::~SideOfRelationFunction() {
  mp_obstacle = NULL;
}

vector< pair<ReferenceFrame*, bool> > SideOfRelationFunction::get_reference_frames( ReferenceFrameSet* p_reference_frame_set ) {
  vector< pair<ReferenceFrame*, bool> > reference_frames;

  return reference_frames;  
}

string SideOfRelationFunction::get_name() {
  string name = "";
  switch( m_type ) {
    default:
      name = "UNKNOWN";
      break;
    case SIDE_TYPE_LEFT:
      name = "LEFT";
      break;
    case SIDE_TYPE_RIGHT:
      name = "RIGHT";
      break;
    case SIDE_TYPE_TOP:
      name = "TOP";
      break;
    case SIDE_TYPE_BOTTOM:
      name = "BOTTOM";
      break;
  }
  return name;
}
