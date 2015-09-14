#include "HASiRRTstar.h"



HASiRRTstar::HASiRRTstar( int width, int height, int segment_length )
            : HARRTstar( width, height, segment_length ) {

}

HASiRRTstar::~HASiRRTstar() {

}

void HASiRRTstar::extend() {
  RRTNode* p_st_new_node = _extend(START_TREE_TYPE);
  Path* p_st_new_path = find_path( p_st_new_node->m_pos );

  if( p_st_new_path ) {
    _p_string_class_mgr->import_path( p_st_new_path );
  }
}
 
