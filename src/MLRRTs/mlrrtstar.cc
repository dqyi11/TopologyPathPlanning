#include "mlrrtstar.h"

using namespace homotopy;
using namespace mlrrts;

MLRRTNode::MLRRTNode(POS2D pos) {
  m_pos = pos;
  m_cost = 0.0;
  mp_parent = NULL;
  m_child_nodes.clear();
  m_substring.clear();
}

bool MLRRTNode::operator==(const MLRRTNode &other) {
  return m_pos==other.m_pos;
}

void MLRRTNode::clear_string() {
  m_substring.clear();
}

void MLRRTNode::append_to_string( std::vector< std::string > ids ) {
  for( unsigned int i = 0; i < ids.size(); i ++ ) {
    std::string id = ids[i];
    m_substring.push_back( id );
  }
}

Path::Path(POS2D start, POS2D goal) {
  m_start = start;
  m_goal = goal;
  m_cost = 0.0;

  m_way_points.clear();
  m_string.clear();
}

Path::~Path() {
  m_cost = 0.0;
}

void Path::append_waypoints( std::vector<POS2D> waypoints, bool reverse ) {
  if ( reverse ) {
    for( std::vector<POS2D>::reverse_iterator itr = waypoints.rbegin();
         itr != waypoints.rend(); itr++ ) {
      POS2D pos = (*itr);
      m_way_points.push_back( pos );
    }
  }
  else {
    for( std::vector<POS2D>::iterator it = waypoints.begin();
         it != waypoints.end(); it++ ) {
      POS2D pos = (*it);
      m_way_points.push_back( pos );
    }
  }
}

void Path::append_substring( std::vector< std::string > ids, bool reverse ) {
  if ( reverse ) {
    for( std::vector< std::string >::reverse_iterator itr = ids.rbegin();
         itr != ids.rend(); itr++ ) {
      std::string str = (*itr);
      m_string.push_back( str );
    }
  }
  else {
    for( std::vector< std::string >::iterator it = ids.begin();
         it != ids.end(); it++ ) {
      std::string str = (*it);
      m_string.push_back( str );
    }
  }
}

MLRRTstar::MLRRTstar( int width, int height, int segment_length ) {

}

MLRRTstar::~MLRRTstar() {

}

bool MLRRTstar::init( POS2D start, POS2D goal, COST_FUNC_PTR p_func, double** pp_cost_distribution, homotopy::grammar_type_t grammar_type ) {

  return false;
}

void MLRRTstar::extend() {

}

void MLRRTstar::set_reference_frames( ReferenceFrameSet* p_reference_frames ) {
  _reference_frames = p_reference_frames;
}

Path* MLRRTstar::find_path( POS2D via_pos ) {
  Path* p_path = NULL;
  return p_path;
}

std::vector<Path*> MLRRTstar::get_paths() {
  std::vector<Path*> paths;
  return paths;
}
