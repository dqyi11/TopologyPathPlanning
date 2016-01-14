#include "cgal_util.h"
#include "sideof_relation_function.h"

using namespace std;
using namespace homotopy;
using namespace topology_inference;

#define PI 3.14159265

SideOfRelationFunction::SideOfRelationFunction( side_type_t side_type ) {
  m_type = side_type;
  mp_obstacle = NULL;
}

SideOfRelationFunction::~SideOfRelationFunction() {
  mp_obstacle = NULL;
}

vector< pair<ReferenceFrame*, bool> > SideOfRelationFunction::get_rules( ReferenceFrameSet* p_reference_frame_set ) {
  vector< pair<ReferenceFrame*, bool> > rules;
  if( p_reference_frame_set ) {
    for( unsigned int i=0; i<p_reference_frame_set->get_reference_frames().size(); i++ ) {
      ReferenceFrame* p_ref = p_reference_frame_set->get_reference_frames()[i];
      if( p_ref ) {
        if( p_ref->mp_line_subsegment && p_ref->mp_line_subsegment->is_connected( mp_obstacle ) ) {
          double radius = get_radius( mp_obstacle->get_centroid(), p_ref->m_mid_point );         
          switch( m_type ) {
          default:
            break;
          case SIDE_TYPE_LEFT:
            // [0.75PI, PI] [-PI, -0.75PI) 
            if( (radius>=0.75*PI && radius<PI) || (radius>=-PI && radius<-0.75*PI) ) {
              rules.push_back( make_pair( p_ref, true ) );
            }
            break;
          case SIDE_TYPE_RIGHT:
            // [-0.45PI, 0.45PI)
            if( radius>=-0.45*PI && radius<0.45*PI ) {
              rules.push_back( make_pair( p_ref, true ) );
            }
            break;
          case SIDE_TYPE_TOP:
            // [-0.75PI, -045PI)
            if( radius>=-0.75*PI && radius<-0.45*PI ) {
              rules.push_back( make_pair( p_ref, true ) );
            }
            break;
          case SIDE_TYPE_BOTTOM:
            // [0.45PI , 0.75PI)
            if( radius>=0.45*PI && radius<0.75*PI ) {
              rules.push_back( make_pair( p_ref, true ) );
            }
            break;
          } 
        }
      }
    }
  }
  return rules;  
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
  if( mp_obstacle ) {
    name += "(" + mp_obstacle->get_name() + ")";
  }
  return name;
}
