#include "mlrrtstar.h"
#include "ml_util.h"

using namespace std;
using namespace homotopy;
using namespace mlrrts;

#define OBSTACLE_THRESHOLD 200

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
  bool node_inserted = false;
  while( false == node_inserted ) {
    POS2D rnd_pos = _sampling();
    KDNode2D nearest_node = _find_nearest( rnd_pos );
   
    if (rnd_pos[0]==nearest_node[0] && rnd_pos[1]==nearest_node[1]) {
      continue;
    }
 
    POS2D new_pos = _steer( rnd_pos, nearest_node ); 
    if( true == _contains(new_pos) ) {
      continue;
    }
    if( true == _is_in_obstacle( new_pos ) ) {
      continue;
    }

    if( true == _is_obstacle_free( nearest_node, new_pos ) ) {
       SubRegion* p_subregion = _reference_frames->get_world_map()->in_subregion( toPoint2D( new_pos ) );
       if (p_subregion) {
         SubRegionMgr* p_mgr = _p_expanding_tree_mgr->find_subregion_mgr( p_subregion );
         for( std::vector<ExpandingNode*>::iterator it = p_mgr->mp_nodes.begin();
              it != p_mgr->mp_nodes.end(); it ++ ) {
           ExpandingNode* p_node = (*it);
           if( p_node ) {
                
           }
         }
       }
    }
  }
}

POS2D MLRRTstar::_sampling() {
  double x = rand();
  double y = rand();
  int int_x = x * ((double)(_sampling_width)/RAND_MAX);
  int int_y = y * ((double)(_sampling_height)/RAND_MAX);
  POS2D m(int_x, int_y);
  return m;
}

POS2D MLRRTstar::_steer( POS2D pos_a, POS2D pos_b ) {
  POS2D new_pos( pos_a[0], pos_a[1] );
  double delta[2];
  delta[0] = pos_a[0] - pos_b[0];
  delta[1] = pos_a[1] - pos_b[1];
  double delta_len = sqrt(delta[0]*delta[0] + delta[1]*delta[1]);
  if ( delta_len > _segment_length ) {
    double scale = _segment_length / delta_len;
    delta[0] = delta[0] * scale;
    delta[1] = delta[1] * scale;
   
    new_pos.setX( pos_b[0]+delta[0] );
    new_pos.setY( pos_b[1]+delta[1] );
  }
  return new_pos;
}

bool MLRRTstar::_is_in_obstacle( POS2D pos ) {
  int x = (int)pos[0];
  int y = (int)pos[1];
  if( _pp_map_info[x][y] < 255 ) {
    return true;
  }
  return false;
}

bool MLRRTstar::_is_obstacle_free( POS2D pos_a, POS2D pos_b ) {
  if ( pos_a == pos_b ) {
    return true;
  }
  int x_dist = pos_a[0] - pos_b[0];
  int y_dist = pos_a[1] - pos_b[1];

  if( x_dist == 0 && y_dist == 0) {
    return true;
  }

  float x1 = pos_a[0];
  float y1 = pos_a[1];
  float x2 = pos_b[0];
  float y2 = pos_b[1];

  const bool steep = ( fabs(y2 - y1) > fabs(x2 - x1) );
  if ( steep ) {
    std::swap( x1, y1 );
    std::swap( x2, y2 );
  }

  if ( x1 > x2 ) {
    std::swap( x1, x2 );
    std::swap( y1, y2 );
  }

  const float dx = x2 - x1;
  const float dy = fabs( y2 - y1 );

  float error = dx / 2.0f;
  const int ystep = (y1 < y2) ? 1 : -1;
  int y = (int)y1;

  const int maxX = (int)x2;

  for(int x=(int)x1; x<maxX; x++) {
    if(steep) {
      if ( y>=0 && y<_sampling_width && x>=0 && x<_sampling_height ) {
        if ( _pp_map_info[y][x] < OBSTACLE_THRESHOLD ) {
          return false;
        }
      }
    }
    else {
      if ( x>=0 && x<_sampling_width && y>=0 && y<_sampling_height ) {
        if ( _pp_map_info[x][y] < OBSTACLE_THRESHOLD ) {
          return false;
        }
      }
    }

    error -= dy;
    if(error < 0) {
      y += ystep;
      error += dx;
    }
  }
  return true;
}

KDNode2D MLRRTstar::_find_nearest( POS2D pos ) {
  KDNode2D node( pos );
  std::pair<KDTree2D::const_iterator,double> found = _p_kd_tree->find_nearest( node );
  KDNode2D near_node = *found.first;
  return near_node;
}

bool MLRRTstar::_contains( POS2D pos ) {
  if(_p_kd_tree) {
    KDNode2D node( pos[0], pos[1] );
    KDTree2D::const_iterator it = _p_kd_tree->find(node);
    if( it!=_p_kd_tree->end() ) {
      return true;
    }
    else {
      return false;
    }
  }
  return false;
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

void MLRRTstar::init_feasible_paths() {
  if( _p_expanding_tree_mgr ) {
    ExpandingTree * p_expanding_tree = _p_expanding_tree_mgr->mp_expanding_tree;
    if( p_expanding_tree ) {
      for( std::vector<ExpandingEdge*>::iterator it = p_expanding_tree->m_edges.begin();
           it != p_expanding_tree->m_edges.end(); it++ ) {
        ExpandingEdge* p_edge = (*it);
        p_edge->m_rand_pos = p_edge->sample_random_pos();
      } 

      for( std::vector<ExpandingNode*>::iterator it = p_expanding_tree->m_nodes.begin();
           it != p_expanding_tree->m_nodes.end(); it++ ) {
        ExpandingNode* p_node = (*it);
        for( std::vector<ExpandingEdge*>::iterator ito = p_node->mp_out_edges.begin();
             ito != p_node->mp_out_edges.end(); ito ++ ) {
          ExpandingEdge* p_out_edge = (*ito);
          vector<POS2D> feasible_path = p_node->find_feasible_path( p_node->mp_in_edge, p_out_edge );
          /* add the set of POS2D as new nodes */
          /* TODO */
        }
      } 
    }
  }

}
