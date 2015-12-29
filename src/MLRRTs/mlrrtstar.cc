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
  mp_master = NULL;
  m_child_nodes.clear();
  m_substring.clear();
}

bool MLRRTNode::operator==(const MLRRTNode &other) {
  if( m_pos==other.m_pos && mp_parent == other.mp_parent ) {
    return true;
  }
  return false;
}

void MLRRTNode::clear_string() {
  m_substring.clear();
}

void MLRRTNode::append_to_string( vector< string > ids ) {
  for( unsigned int i = 0; i < ids.size(); i ++ ) {
    string id = ids[i];
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

void Path::append_waypoints( vector<POS2D> waypoints, bool reverse ) {
  if ( reverse ) {
    for( vector<POS2D>::reverse_iterator itr = waypoints.rbegin();
         itr != waypoints.rend(); itr++ ) {
      POS2D pos = (*itr);
      m_way_points.push_back( pos );
    }
  }
  else {
    for( vector<POS2D>::iterator it = waypoints.begin();
         it != waypoints.end(); it++ ) {
      POS2D pos = (*it);
      m_way_points.push_back( pos );
    }
  }
}

void Path::append_substring( vector< string > ids, bool reverse ) {
  if ( reverse ) {
    for( vector< string >::reverse_iterator itr = ids.rbegin();
         itr != ids.rend(); itr++ ) {
      string str = (*itr);
      m_string.push_back( str );
    }
  }
  else {
    for( vector< string >::iterator it = ids.begin();
         it != ids.end(); it++ ) {
      string str = (*it);
      m_string.push_back( str );
    }
  }
}

MLRRTstar::MLRRTstar( int width, int height, int segment_length ) {
  _sampling_width = width;
  _sampling_height = height;
  _segment_length = segment_length;

  _p_root = NULL;
  _reference_frames = NULL;

  _grammar_type = STRING_GRAMMAR_TYPE;
  _p_master_kd_tree = new KDTree2D( ptr_fun(tac) );
 
  _range = (_sampling_width > _sampling_height) ? _sampling_width : _sampling_height;
  _obs_check_resolution = 1;
  _current_iteration = 0;
  _homotopic_enforcement = false;

  _theta = 10;
  _pp_cost_distribution = NULL;
  _p_expanding_tree_mgr = NULL;

  _pp_map_info = new int*[_sampling_width];
  for(int i=0;i<_sampling_width;i++) {
    _pp_map_info[i] = new int[_sampling_height];
    for(int j=0;j<_sampling_height;j++) {
      _pp_map_info[i][j] = 255;
    }
  }    
}

MLRRTstar::~MLRRTstar() {
  if( _p_master_kd_tree ) {
    delete _p_master_kd_tree;
    _p_master_kd_tree = NULL;
  }
  if (_pp_map_info) {
    for(int i=0;i<_sampling_width;i++) {
      delete _pp_map_info[i];
      _pp_map_info[i] = NULL;
    }
    delete _pp_map_info;
    _pp_map_info = NULL;
  }
}

bool MLRRTstar::init( POS2D start, POS2D goal, COST_FUNC_PTR p_func, double** pp_cost_distribution, homotopy::grammar_type_t grammar_type ) {
  if( _p_root ) {
    delete _p_root;
    _p_root = NULL;
  }
  if( _p_expanding_tree_mgr ) {
    delete _p_expanding_tree_mgr;
    _p_expanding_tree_mgr = NULL;
  }
  if (_reference_frames == NULL) {
    return false;
  }

  _start = start;
  _goal = goal;
  _p_cost_func = p_func;

  if(pp_cost_distribution) {
    if(_pp_cost_distribution == NULL) {
      _pp_cost_distribution = new double*[_sampling_width];
      for(int i=0;i<_sampling_width;i++) {
        _pp_cost_distribution[i] = new double[_sampling_height];
      }
    }
    for(int i=0;i<_sampling_width;i++) {
      for(int j=0;j<_sampling_height;j++) {
        _pp_cost_distribution[i][j] = pp_cost_distribution[i][j];
      }
    }
  }

  cout << "Init grammar ... " << endl; 
  Point2D start_point = toPoint2D( _start );
  Point2D goal_point = toPoint2D( _goal );
  set_grammar_type(grammar_type);
  if( STRING_GRAMMAR_TYPE == grammar_type) {
    _string_grammar = _reference_frames->get_string_grammar( start_point, goal_point );
  }
  else if( HOMOTOPIC_GRAMMAR_TYPE == grammar_type ) {
    _string_grammar = _reference_frames->get_homotopic_grammar(start_point, goal_point );
  }
  cout << "Init String Class Mgr ... " << endl;
  _p_expanding_tree_mgr = new ExpandingTreeMgr();
  _p_expanding_tree_mgr->init( _string_grammar, _reference_frames );

  cout << "Init tree.." << endl;
  KDNode2D root( start );
  _p_root = new MLRRTNode( start );
  _nodes.push_back(_p_root);
  root.add_mlrrtnode(_p_root);
  _p_master_kd_tree->insert( root );
  SubRegion* p_subregion = _reference_frames->get_world_map()->in_subregion( toPoint2D( start ) );
  if (p_subregion) {
    SubRegionMgr* p_mgr = _p_expanding_tree_mgr->find_subregion_mgr( p_subregion );
    if(p_mgr) {
      for( vector<ExpandingNode*>::iterator it = p_mgr->mp_nodes.begin();
           it != p_mgr->mp_nodes.end(); it ++ ) {
        ExpandingNode* p_exp_node = (*it);
        if(p_exp_node) {
          for( vector<StringClass*>::iterator it_str_cls = p_exp_node->mp_string_classes.begin();
               it_str_cls != p_exp_node->mp_string_classes.end(); it_str_cls++ ) {
            StringClass* p_str_cls = (*it_str_cls);
            if( p_str_cls ) {
              if( p_str_cls->mp_kd_tree ) {
                KDNode2D new_node( start );
                new_node.set_pri_mlrrtnode( _p_root );
                p_str_cls->mp_kd_tree->insert( new_node );
              }
            }
          }
        }
      }
    }
  } 
   
  _current_iteration = 0; 
  
  return true;
}

void MLRRTstar::extend() {
  bool node_inserted = false;
  while( false == node_inserted ) {
    POS2D rnd_pos = _sampling();
    KDNode2D nearest_node = _find_nearest( rnd_pos, NULL );
   
    if ( rnd_pos[0]==nearest_node[0] && rnd_pos[1]==nearest_node[1] ) {
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
       //cout << "NEW POS " << new_pos << endl;
       SubRegion* p_subregion = _reference_frames->get_world_map()->in_subregion( toPoint2D( new_pos ) );
       if ( p_subregion ) {
         SubRegionMgr* p_mgr = _p_expanding_tree_mgr->find_subregion_mgr( p_subregion );
         if( p_mgr ) {
           KDNode2D new_master_node( new_pos );
           bool any_node_added = false;
           /* EACH EXPANDING NODE OF THE NEW POS */
           for( vector<ExpandingNode*>::iterator it = p_mgr->mp_nodes.begin();
                it != p_mgr->mp_nodes.end(); it ++ ) {
             ExpandingNode* p_exp_node = (*it);
             if( p_exp_node ) {
               // create new node 
               MLRRTNode* p_new_rnode = _create_new_node( new_pos, p_exp_node ); 
               KDNode2D nearest_node_in_class = _find_nearest( new_pos, p_exp_node );  
               list<KDNode2D> near_list_in_class = _find_near( new_pos, p_exp_node );

               //MLRRTNode* p_nearest_rnode = nearest_node_in_class.get_pri_mlrrtnode();
               list<MLRRTNode*> near_rnodes;
               near_rnodes.clear();
               for( list<KDNode2D>::iterator itr = near_list_in_class.begin();
                    itr != near_list_in_class.end(); itr ++ ) {
                 KDNode2D near_kd_node = (*itr);
                 MLRRTNode* p_near_rnode = near_kd_node.get_pri_mlrrtnode();
                 near_rnodes.push_back( p_near_rnode );
               } 
               // attach new noue 
               if( _attach_new_node( p_new_rnode, near_rnodes, p_exp_node ) ) {
                 any_node_added = true;
                 new_master_node.add_mlrrtnode( p_new_rnode );
                 p_new_rnode->mp_master = p_exp_node;              
  
                 if( p_exp_node ) {
                   for( vector<StringClass*>::iterator it_str_cls = p_exp_node->mp_string_classes.begin();
                        it_str_cls != p_exp_node->mp_string_classes.end(); it_str_cls++ ) {
                     StringClass* p_str_cls = (*it_str_cls);
                     if( p_str_cls->mp_kd_tree ) {
                       KDNode2D new_node( new_pos );
                       new_node.set_pri_mlrrtnode( p_new_rnode );
                       p_str_cls->mp_kd_tree->insert( new_node );
                     }
                   }
                   p_exp_node->mp_nodes.push_back( p_new_rnode );
                 }
               }
               // rewire near nodes    
               _rewire_near_nodes( p_new_rnode, near_rnodes, p_exp_node );      
            }
          }  
          if ( any_node_added ) {
             _p_master_kd_tree->insert( new_master_node );
             node_inserted = true;
          }
        }
      }
    }
  }
  _current_iteration ++;
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
    swap( x1, y1 );
    swap( x2, y2 );
  }

  if ( x1 > x2 ) {
    swap( x1, x2 );
    swap( y1, y2 );
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

KDNode2D MLRRTstar::_find_nearest( POS2D pos, ExpandingNode* p_exp_node ) {
  KDNode2D node( pos );
  KDNode2D nearest_node( pos );
  if( p_exp_node == NULL ) {
    pair<KDTree2D::const_iterator,double> found = _p_master_kd_tree->find_nearest( node );
    nearest_node = *found.first;
  }
  else {
    /* find nearest in each string class */
    double nearest_distance = _sampling_width > _sampling_height ? _sampling_width : _sampling_height;
    
    for(vector<StringClass*>::iterator it = p_exp_node->mp_string_classes.begin(); it != p_exp_node->mp_string_classes.end(); it ++ ) {
      StringClass* p_class = (*it);
      pair<KDTree2D::const_iterator,double> found = p_class->mp_kd_tree->find_nearest( node );
      KDNode2D nearest_node_in_class = *found.first;
      double distance_in_class = found.second;  
      
      if( (distance_in_class < nearest_distance) &&
           in_current_and_parent_exp_node( nearest_node, p_exp_node ) ) {
        nearest_distance = distance_in_class;
        nearest_node = nearest_node_in_class;
      }
    } 
    
  } 
  return nearest_node;
}

list<KDNode2D> MLRRTstar::_find_near( POS2D pos, ExpandingNode* p_exp_node ) {
  list<KDNode2D> near_list;
  KDNode2D node(pos);
  int num_dimensions = 2;
  if( p_exp_node == NULL ) {  
    int num_vertices = _p_master_kd_tree->size();
    double ball_radius =  _theta * _range * pow( log((double)(num_vertices + 1.0))/((double)(num_vertices + 1.0)), 1.0/((double)num_dimensions) );

    _p_master_kd_tree->find_within_range( node, ball_radius, back_inserter( near_list ) );
  }
  else {

    for(vector<StringClass*>::iterator it = p_exp_node->mp_string_classes.begin(); it != p_exp_node->mp_string_classes.end(); it ++ ) {
      StringClass* p_class = (*it);
      list<KDNode2D> near_list_in_class;
      int num_vertices = p_class->mp_kd_tree->size();
      double ball_radius =  _theta * _range * pow( log((double)(num_vertices + 1.0))/((double)(num_vertices + 1.0)), 1.0/((double)num_dimensions) );
      p_class->mp_kd_tree->find_within_range( node, ball_radius, back_inserter( near_list_in_class ) );
      //cout << "NEAR LIST IN CLASS " << near_list_in_class.size() << endl;
      for( list<KDNode2D>::iterator it_cls = near_list_in_class.begin();
           it_cls != near_list_in_class.end(); it_cls ++ ) {
        KDNode2D kdnode = (*it_cls);
        if ( in_current_and_parent_exp_node( kdnode, p_exp_node ) ) {
          near_list.push_back( kdnode );
        }
      }      
    }
  }
  //cout << "NEAR LIST " << near_list.size() << endl;
  return near_list;
}

bool MLRRTstar::in_current_and_parent_exp_node( POS2D pos, ExpandingNode* p_exp_node ) {
  if( p_exp_node ) {
    if( p_exp_node->mp_subregion ) {
      if( p_exp_node->mp_subregion->contains( toPoint2D( pos ) ) ) {
        return true;
      }
    } 
    if( p_exp_node->mp_in_edge ){
      if( p_exp_node->mp_in_edge->mp_from ){
        if( p_exp_node->mp_in_edge->mp_from->mp_subregion ) {
          if( p_exp_node->mp_in_edge->mp_from->mp_subregion->contains( toPoint2D( pos ) ) ) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

bool MLRRTstar::_contains( POS2D pos ) {
  if(_p_master_kd_tree) {
    KDNode2D node( pos[0], pos[1] );
    KDTree2D::const_iterator it = _p_master_kd_tree->find(node);
    if( it!=_p_master_kd_tree->end() ) {
      return true;
    }
    else {
      return false;
    }
  }
  return false;
}

MLRRTNode* MLRRTstar::_create_new_node( POS2D pos, ExpandingNode* p_exp_node ) {
  MLRRTNode* p_node = new MLRRTNode( pos );
  _nodes.push_back( p_node );
  if( p_exp_node ) {
    p_exp_node->mp_nodes.push_back( p_node );
  }
  return p_node;
}

void MLRRTstar::set_reference_frames( ReferenceFrameSet* p_reference_frames ) {
  _reference_frames = p_reference_frames;
}

vector<Path*> MLRRTstar::get_paths() {
  vector<Path*> paths;
  
  if( _p_expanding_tree_mgr ) {
    vector<StringClass*> string_classes = _p_expanding_tree_mgr->get_string_classes();
    for( vector<StringClass*>::iterator it = string_classes.begin(); 
         it != string_classes.end(); it ++ ) {
      StringClass* p_string_class = (*it);
      Path* p_path = _get_path( p_string_class );
      p_string_class->mp_path = p_path;
      paths.push_back( p_path );
    }
  }

  return paths;
}

Path* MLRRTstar::_get_path( StringClass* p_string_class ) {

  if( p_string_class ) {
    int exp_node_num = p_string_class->mp_exp_nodes.size();
    ExpandingNode* p_last_exp_node = p_string_class->mp_exp_nodes[exp_node_num-1];
    if( p_last_exp_node ) {
      KDNode2D kdnode = _find_nearest( _goal, p_last_exp_node );
      MLRRTNode* p_near_goal = kdnode.get_pri_mlrrtnode();
      if( p_near_goal ) {
        Path* p_path = _get_path( p_near_goal );
        return p_path;
      }
    }
  }
  return NULL;
}

Path* MLRRTstar::_get_path( MLRRTNode* p_node ) {
  list<MLRRTNode*> node_list;
  get_parent_node_list( p_node, node_list );
  Path* p_path = new Path( _start, _goal ); 
  for( list<MLRRTNode*>::reverse_iterator itr = node_list.rbegin();
       itr != node_list.rend(); itr ++ ) {
    MLRRTNode* p_rrt_node = (*itr);
    p_path->m_way_points.push_back( p_rrt_node->m_pos );
  }
  p_path->m_way_points.push_back( _goal );
  p_path->append_substring( p_node->m_substring );
  p_path->m_cost = p_node->m_cost;
  return p_path;
}

void MLRRTstar::init_feasible_paths() {
  if( _p_expanding_tree_mgr ) {
    ExpandingTree * p_expanding_tree = _p_expanding_tree_mgr->mp_expanding_tree;
    if( p_expanding_tree ) {
      for( vector<ExpandingEdge*>::iterator it = p_expanding_tree->m_edges.begin();
           it != p_expanding_tree->m_edges.end(); it++ ) {
        ExpandingEdge* p_edge = (*it);
        p_edge->m_rand_pos = p_edge->sample_random_pos();
      } 

      for( vector<ExpandingNode*>::iterator it = p_expanding_tree->m_nodes.begin();
           it != p_expanding_tree->m_nodes.end(); it++ ) {
        ExpandingNode* p_node = (*it);
        for( vector<ExpandingEdge*>::iterator ito = p_node->mp_out_edges.begin();
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

bool MLRRTstar::_attach_new_node( MLRRTNode* p_node_new, list<MLRRTNode*> near_nodes, ExpandingNode* p_exp_node ) {
  double min_new_node_cost = -1;
  MLRRTNode* p_min_node = NULL;

  for(list<MLRRTNode*>::iterator it = near_nodes.begin(); it != near_nodes.end(); it++) {
    MLRRTNode* p_near_node = (*it);
    if( true == _is_obstacle_free( p_near_node->m_pos, p_node_new->m_pos ) ) {
      bool eligible = true;
      if( _homotopic_enforcement ) {
        eligible = _is_homotopic_constrained( p_near_node, p_node_new, p_exp_node );
      } 
      if( eligible ) { 
        double delta_cost = _calculate_cost( p_near_node->m_pos, p_node_new->m_pos );
        double new_cost = p_near_node->m_cost + delta_cost;
        if( (p_min_node==NULL) || (new_cost < min_new_node_cost) ) {
          p_min_node = p_near_node;
          min_new_node_cost = new_cost;
        }
      }
    }
  } 
  if( p_min_node == NULL ) {
    return false;
  }
 
  bool added = _add_edge( p_min_node, p_node_new );
  if( added ) {
    p_node_new->m_cost = min_new_node_cost;
    return true;
  }
  return false;
}

void MLRRTstar::_rewire_near_nodes( MLRRTNode* p_node_new, list<MLRRTNode*> near_nodes, ExpandingNode* p_exp_node ) {
  for( list<MLRRTNode*>::iterator it = near_nodes.begin();
       it != near_nodes.end(); it++ ) {
    MLRRTNode* p_near_node = (*it);

    if( p_near_node->m_pos == p_node_new->m_pos ||
        p_near_node->m_pos == _p_root->m_pos ) {
      continue;
    }

    if( true == _is_obstacle_free( p_node_new->m_pos, p_near_node->m_pos ) ) {
      bool eligible = true;
      if( _homotopic_enforcement ) {
        eligible = _is_homotopic_constrained( p_node_new, p_near_node, p_exp_node ); 
      }
      if( eligible ) {
        double temp_delta_cost = _calculate_cost( p_node_new->m_pos, p_near_node->m_pos );
        double temp_cost_from_new_node = p_node_new->m_cost + temp_delta_cost;
        if( temp_cost_from_new_node < p_near_node->m_cost ) {
          double min_delta_cost = p_near_node->m_cost - temp_cost_from_new_node;
          MLRRTNode* p_parent_node = p_near_node->mp_parent;
          bool removed = _remove_edge( p_parent_node, p_near_node );
          if( removed ) {
            bool added = _add_edge( p_node_new, p_near_node );
            if( added ) {
              p_near_node->m_cost = temp_cost_from_new_node;
              _update_cost_to_children( p_near_node, min_delta_cost );
            }
          }
          else {
            std::cout << " Failed in removing " << std::endl;
          }
        }
      }
    }
  }

}

double MLRRTstar::_calculate_cost( POS2D& pos_a, POS2D& pos_b ) {
  return _p_cost_func( pos_a, pos_b, _pp_cost_distribution, this);
}

bool MLRRTstar::_has_edge( MLRRTNode* p_node_parent, MLRRTNode* p_node_child ) {
  if ( p_node_parent == NULL || p_node_child == NULL ) {
    return false;
  }
  for( list<MLRRTNode*>::iterator it=p_node_parent->m_child_nodes.begin();it!=p_node_parent->m_child_nodes.end();it++ ) {
    MLRRTNode* p_curr_node = (*it);
    if( p_curr_node == p_node_child ) {
      return true;
    }
  }
  /*
    if (pNode_p == pNode_c->mpParent)
        return true;
  */
  return false;
}

bool MLRRTstar::_add_edge( MLRRTNode* p_node_parent, MLRRTNode* p_node_child ) {
  if( p_node_parent == NULL || p_node_child == NULL || p_node_parent == p_node_child ) {
    return false;
  }
  if ( p_node_parent->m_pos == p_node_child->m_pos ) {
    return false;
  }
  // generate the string of ID characters
  Point2D start = toPoint2D( p_node_parent->m_pos );
  Point2D goal = toPoint2D( p_node_child->m_pos );
  //std::cout << "START " << start << " END " << goal << std::endl;
  vector< string > ids = _reference_frames->get_string( start, goal, _grammar_type );
  p_node_child->clear_string();
  p_node_child->append_to_string( p_node_parent->m_substring );
  p_node_child->append_to_string( ids );

  if ( true == _has_edge( p_node_parent, p_node_child ) ) {
    p_node_child->mp_parent = p_node_parent;
  }
  else {
    p_node_parent->m_child_nodes.push_back( p_node_child );
    p_node_child->mp_parent = p_node_parent;
  }
  p_node_child->m_child_nodes.unique();

  return true;
}

bool MLRRTstar::_remove_edge( MLRRTNode* p_node_parent, MLRRTNode* p_node_child ) {
  if( p_node_parent==NULL ) {
    return false;
  }

  p_node_child->mp_parent = NULL;
  bool removed = false;
  for( list<MLRRTNode*>::iterator it=p_node_parent->m_child_nodes.begin();it!=p_node_parent->m_child_nodes.end();it++ ) {
    MLRRTNode* p_current = (MLRRTNode*)(*it);
    if ( p_current == p_node_child || p_current->m_pos==p_node_child->m_pos ) {
      p_current->mp_parent = NULL;
      p_current->clear_string();
      it = p_node_parent->m_child_nodes.erase(it);
      removed = true;
    }
  }
  return removed;
}

bool MLRRTstar::_is_homotopic_constrained( MLRRTNode* p_node_parent, MLRRTNode* p_node_child, ExpandingNode* p_exp_node ) {
  Point2D point_a = toPoint2D( p_node_parent->m_pos );
  Point2D point_b = toPoint2D( p_node_child->m_pos );
  if( _reference_frames ) {
    WorldMap* p_world_map = _reference_frames->get_world_map();
    if( p_world_map ) {
      SubRegion* p_subregion_a = p_world_map->in_subregion( point_a );
      SubRegion* p_subregion_b = p_world_map->in_subregion( point_b );
      if( p_subregion_a == p_subregion_b ) {
        //cout << "SAME REGION" << endl;
        return true;
      }
      if( p_exp_node ) {
        if( p_exp_node->mp_in_edge ) {
          if( p_exp_node->mp_in_edge->mp_reference_frame ) {

            if( p_exp_node->mp_in_edge->mp_reference_frame->is_line_crossed( point_a, point_b ) ) {
              //cout << "CROSSED REF" << endl;
              return true;  
            }  
          } 
        }
      }
    }
  }
  return false;
}

void MLRRTstar::_update_cost_to_children( MLRRTNode* p_node, double delta_cost ) {
  list<MLRRTNode*> child_list = _find_all_children( p_node );
  for( list<MLRRTNode*>::iterator it = child_list.begin();
       it != child_list.end(); it++ ) {
    MLRRTNode* p_child_node = (*it);
    if( p_child_node ) {
      p_child_node->m_cost -= delta_cost;
    }
  }
}

list<MLRRTNode*> MLRRTstar::_find_all_children( MLRRTNode* p_node ) {
  int level = 0;
  bool finished = false;
  list<MLRRTNode*> child_list;
  
  list<MLRRTNode*> current_level_nodes;
  current_level_nodes.push_back( p_node );
  while( false == finished ) {
    list<MLRRTNode*> current_level_children;
    int child_list_num = child_list.size();

    for( list<MLRRTNode*>::iterator it = current_level_nodes.begin();
         it != current_level_nodes.end(); it ++ ) {
      MLRRTNode* p_current_node = (*it);
      for( list<MLRRTNode*>::iterator itc = p_current_node->m_child_nodes.begin();
           itc != p_current_node->m_child_nodes.end(); itc ++ ) {
         MLRRTNode* p_child_node = (*itc);
         if( p_child_node ) {
           current_level_children.push_back( p_child_node );
           child_list.push_back( p_child_node );
         }
      }
    }

    child_list.unique();
    current_level_children.unique();

    if( current_level_children.size() == 0 ) {
      finished = true;
    }
    else if( child_list.size() == child_list_num ) {
      finished = true;
    }
    else {
      current_level_nodes.clear();
      for( list<MLRRTNode*>::iterator itt = current_level_children.begin();
           itt != current_level_children.end(); itt ++ ) {
        MLRRTNode* p_temp_node = (*itt);
        if( p_temp_node ) {
          current_level_nodes.push_back( p_temp_node );
        }
      }
      level += 1;
    }

    if( level > 100 ) {
      cout << "LEVEL > 100" << endl;
      break;
    }
  }
  child_list.unique();
  return child_list;
}
