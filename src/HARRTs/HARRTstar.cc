#include <limits>
#include <iostream>
#include <fstream>

#include "HARRTstar.h"

#define OBSTACLE_THRESHOLD 200

RRTNode::RRTNode(POS2D pos) {
  m_pos = pos;
  m_cost = 0.0;
  mp_parent = NULL;
  m_child_nodes.clear();
  m_substring.clear();
}

bool RRTNode::operator==(const RRTNode &other) {
  return m_pos==other.m_pos;
}

void RRTNode::clear_string() {
  m_substring.clear();
}

void RRTNode::append_to_string( std::vector< std::string > ids ) {
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
    for( unsigned int i = waypoints.size()-1; i >= 0; i -- ) {
      m_way_points.push_back( waypoints[i] );  
    }
  }
  else {
    for( unsigned int i = 0; i < waypoints.size(); i ++ )  {
      m_way_points.push_back( waypoints[i] );
    }
  }
}

void Path::append_substring( std::vector< std::string > ids, bool reverse ) {
  if ( reverse ) {
    for( unsigned int i = ids.size()-1; i >= 0; i -- ) {
      m_string.push_back( ids[i] );     
    }
  }
  else {
    for( unsigned int i = 0; i < ids.size(); i ++ )  {
      m_string.push_back( ids[i] );
    }
  }
}

HARRTstar::HARRTstar( int width, int height, int segment_length ) {

  _sampling_width = width;
  _sampling_height = height;
  _segment_length = segment_length;
  _p_st_root = NULL;
  _p_gt_root = NULL;

  _p_st_kd_tree = new KDTree2D( std::ptr_fun(tac) );
  _p_gt_kd_tree = new KDTree2D( std::ptr_fun(tac) );

  _range = (_sampling_width > _sampling_height) ? _sampling_width:_sampling_height;
  _st_ball_radius = _range;
  _gt_ball_radius = _range;
  _obs_check_resolution = 1;
  _current_iteration = 0;
  _segment_length = segment_length;

  _theta = 10;

  _pp_cost_distribution = NULL;
  _p_string_class_mgr = NULL;

  _pp_map_info = new int*[_sampling_width];
  for(int i=0;i<_sampling_width;i++) {
    _pp_map_info[i] = new int[_sampling_height];
    for(int j=0;j<_sampling_height;j++) {
      _pp_map_info[i][j] = 255;
    }
  }

  _st_nodes.clear();
  _gt_nodes.clear();
}

HARRTstar::~HARRTstar() {
  if(_p_st_kd_tree) {
    delete _p_st_kd_tree;
    _p_st_kd_tree = NULL;
  }
}

bool HARRTstar::init( POS2D start, POS2D goal, COST_FUNC_PTR p_func, double** pp_cost_distribution ) {
  if( _p_st_root ) {
    delete _p_st_root;
    _p_st_root = NULL;
  }
  if( _p_gt_root ) {
    delete _p_gt_root;
    _p_gt_root = NULL;
  }
  if( _p_string_class_mgr ) {
    delete _p_string_class_mgr;
    _p_string_class_mgr = NULL;
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
  else {
    if(_pp_cost_distribution) {
      _pp_cost_distribution = NULL;
    }
  }

  Point2D start_point( _start[0], _start[1] );
  Point2D goal_point( _goal[0], _goal[1] );
  _p_string_class_mgr = new StringClassMgr( _reference_frames->get_string_grammar( start_point, goal_point ) );

  KDNode2D st_root( start );
  _p_st_root = new RRTNode( start );
  _st_nodes.push_back(_p_st_root);
  st_root.setRRTNode(_p_st_root);
  _p_st_kd_tree->insert( st_root );
  
  KDNode2D gt_root( start );
  _p_gt_root = new RRTNode( goal );
  _gt_nodes.push_back(_p_gt_root);
  gt_root.setRRTNode(_p_gt_root);
  _p_gt_kd_tree->insert( gt_root );

  _current_iteration = 0;

}

void HARRTstar::load_map( int** pp_map ) {
  for(int i=0;i<_sampling_width;i++) {
    for(int j=0;j<_sampling_height;j++) {
      _pp_map_info[i][j] = pp_map[i][j];
    }
  }
}

POS2D HARRTstar::_sampling() {
  double x = rand();
  double y = rand();
  int int_x = x * ((double)(_sampling_width)/RAND_MAX);
  int int_y = y * ((double)(_sampling_height)/RAND_MAX);

  POS2D m(int_x,int_y);
  return m;
}

POS2D HARRTstar::_steer( POS2D pos_a, POS2D pos_b ) {
  POS2D new_pos( pos_a[0], pos_a[1] );
  double delta[2];
  delta[0] = pos_a[0] - pos_b[0];
  delta[1] = pos_a[1] - pos_b[1];
  double delta_len = sqrt(delta[0]*delta[0]+delta[1]*delta[1]);

  if (delta_len > _segment_length) {
    double scale = _segment_length / delta_len;
    delta[0] = delta[0] * scale;
    delta[1] = delta[1] * scale;

    new_pos.setX( pos_b[0]+delta[0] );
    new_pos.setY( pos_b[1]+delta[1] );
  }
  return new_pos;
}

bool HARRTstar::_is_in_obstacle( POS2D pos ) {
  int x = (int)pos[0];
  int y = (int)pos[1];
  if( _pp_map_info[x][y] < 255 ) {
    return true;
  }
  return false;
}


bool HARRTstar::_is_obstacle_free( POS2D pos_a, POS2D pos_b ) {
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

void HARRTstar::extend() {
  RRTNode* p_st_new_node = _extend(START_TREE_TYPE);
  RRTNode* p_gt_new_node = _extend(GOAL_TREE_TYPE);
  Path* p_st_new_path = find_path( p_st_new_node->m_pos );
  Path* p_gt_new_path = find_path( p_gt_new_node->m_pos ); 

  _p_string_class_mgr->import_path( p_st_new_path );
  _p_string_class_mgr->import_path( p_gt_new_path );

  _current_iteration++;
}

RRTNode* HARRTstar::_extend( RRTree_type_t type ) {
  bool node_inserted = false;
  while( false==node_inserted ) {
    POS2D rnd_pos = _sampling();
    KDNode2D nearest_node = _find_nearest( rnd_pos, type );

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
      std::list<KDNode2D> near_list = _find_near( new_pos, type );
      KDNode2D new_node( new_pos );

      // create new node
      RRTNode * p_new_rnode = _create_new_node( new_pos, type );
      new_node.setRRTNode( p_new_rnode );
      if (type == START_TREE_TYPE) {
        _p_st_kd_tree->insert( new_node );
      }
      else if(type == GOAL_TREE_TYPE) {
        _p_gt_kd_tree->insert( new_node );
      }
      node_inserted = true;

      RRTNode* p_nearest_rnode = nearest_node.getRRTNode();
      std::list<RRTNode*> near_rnodes;
      near_rnodes.clear();
      for( std::list<KDNode2D>::iterator itr = near_list.begin();
           itr != near_list.end(); itr++ ) {
        KDNode2D kd_node = (*itr);
        RRTNode* p_near_rnode = kd_node.getRRTNode();
        near_rnodes.push_back( p_near_rnode );
      }
      // attach new node to reference trees
      _attach_new_node( p_new_rnode, p_nearest_rnode, near_rnodes );
      // rewire near nodes of reference trees
      _rewire_near_nodes( p_new_rnode, near_rnodes );

      return p_new_rnode;
    }
  }
  return NULL;
}

KDNode2D HARRTstar::_find_nearest( POS2D pos, RRTree_type_t type ) {
  KDNode2D node( pos );
  if( START_TREE_TYPE == type) {
    std::pair<KDTree2D::const_iterator,double> found = _p_st_kd_tree->find_nearest( node );
    KDNode2D near_node = *found.first;
    return near_node;
  }
  else if( GOAL_TREE_TYPE == type ) {
    std::pair<KDTree2D::const_iterator,double> found = _p_gt_kd_tree->find_nearest( node );
    KDNode2D near_node = *found.first;
    return near_node;
  }
  return node;
}

std::list<KDNode2D> HARRTstar::_find_near( POS2D pos, RRTree_type_t type ) {
  std::list<KDNode2D> near_list;
  KDNode2D node(pos);

  int num_dimensions = 2;
  if ( START_TREE_TYPE == type ) {
    int num_vertices = _p_st_kd_tree->size();
    _st_ball_radius =  _theta * _range * pow( log((double)(num_vertices + 1.0))/((double)(num_vertices + 1.0)), 1.0/((double)num_dimensions) );

    _p_st_kd_tree->find_within_range( node, _st_ball_radius, std::back_inserter( near_list ) );
  }
  else if ( GOAL_TREE_TYPE == type ) {
    int num_vertices = _p_gt_kd_tree->size();
    _gt_ball_radius =  _theta * _range * pow( log((double)(num_vertices + 1.0))/((double)(num_vertices + 1.0)), 1.0/((double)num_dimensions) );

    _p_gt_kd_tree->find_within_range( node, _gt_ball_radius, std::back_inserter( near_list ) );
  }
  return near_list;
}


bool HARRTstar::_contains( POS2D pos ) {
  if(_p_st_kd_tree) {
    KDNode2D node( pos[0], pos[1] );
    KDTree2D::const_iterator it = _p_st_kd_tree->find(node);
    if( it!=_p_st_kd_tree->end() ) {
      return true;
    }
    else {
      return false;
    }
  }
  return false;
}

double HARRTstar::_calculate_cost( POS2D& pos_a, POS2D& pos_b ) {
  return _p_cost_func(pos_a, pos_b, _pp_cost_distribution, this);
}

RRTNode* HARRTstar::_create_new_node(POS2D pos, RRTree_type_t type) {
  RRTNode * pNode = new RRTNode(pos);
  if( type == START_TREE_TYPE ) {
    _st_nodes.push_back(pNode);
  }
  else if( type == GOAL_TREE_TYPE ) {
    _gt_nodes.push_back(pNode);
  }
  return pNode;
}

bool HARRTstar::_remove_edge( RRTNode* p_node_parent, RRTNode*  p_node_child ) {
  if( p_node_parent==NULL ) {
    return false;
  }

  p_node_child->mp_parent = NULL;
  bool removed = false;
  for( std::list<RRTNode*>::iterator it=p_node_parent->m_child_nodes.begin();it!=p_node_parent->m_child_nodes.end();it++ ) {
    RRTNode* p_current = (RRTNode*)(*it);
    if ( p_current == p_node_child || p_current->m_pos==p_node_child->m_pos ) {
      p_current->mp_parent = NULL;
      p_current->clear_string();
      it = p_node_parent->m_child_nodes.erase(it);
      removed = true;
    }
  }
  return removed;
}

bool HARRTstar::_has_edge(RRTNode* p_node_parent, RRTNode* p_node_child) {
  if ( p_node_parent == NULL || p_node_child == NULL ) {
    return false;
  }
  for( std::list<RRTNode*>::iterator it=p_node_parent->m_child_nodes.begin();it!=p_node_parent->m_child_nodes.end();it++ ) {
    RRTNode* p_curr_node = (*it);
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

bool HARRTstar::_add_edge( RRTNode* p_node_parent, RRTNode* p_node_child ) {
  if( p_node_parent == NULL || p_node_child == NULL || p_node_parent == p_node_child ) {
    return false;
  }
  if ( p_node_parent->m_pos == p_node_child->m_pos ) {
    return false;
  }
  // generate the string of ID characters
  Point2D start = Point2D( p_node_parent->m_pos[0],
                           p_node_parent->m_pos[1] );
  Point2D goal = Point2D( p_node_child->m_pos[0],
                          p_node_child->m_pos[1] );
  std::vector< std::string > ids = _reference_frames->get_string( start, goal, STRING_GRAMMAR_TYPE );
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


std::list<RRTNode*> HARRTstar::_find_all_children( RRTNode* p_node ) {
  int level = 0;
  bool finished = false;
  std::list<RRTNode*> child_list;

  std::list<RRTNode*> current_level_nodes;
  current_level_nodes.push_back( p_node );
  while( false==finished ) {
    std::list<RRTNode*> current_level_children;
    int child_list_num = child_list.size();

    for( std::list<RRTNode*>::iterator it=current_level_nodes.begin(); it!=current_level_nodes.end(); it++ ) {
      RRTNode* pCurrentNode = (*it);
      for( std::list<RRTNode*>::iterator itc=pCurrentNode->m_child_nodes.begin(); itc!=pCurrentNode->m_child_nodes.end();itc++ ) {
        RRTNode *p_child_node= (*itc);
        if(p_child_node) {
          current_level_children.push_back(p_child_node);
          child_list.push_back(p_child_node);
        }
      }
    }

    child_list.unique();
    current_level_children.unique();

    if (current_level_children.size()==0) {
      finished = true;
    }
    else if (child_list.size()==child_list_num) {
      finished = true;
    }
    else {
      current_level_nodes.clear();
      for( std::list<RRTNode*>::iterator itt=current_level_children.begin();itt!=current_level_children.end();itt++ ) {
        RRTNode * pTempNode = (*itt);
        if( pTempNode ) {
          current_level_nodes.push_back( pTempNode );
        }
      }
      level +=1;
    }

    if(level>100) {
      break;
    }
  }
  child_list.unique();
  return child_list;
}


RRTNode* HARRTstar::_find_ancestor(RRTNode* p_node) {
  return get_ancestor( p_node );
}

Path* HARRTstar::find_path( POS2D via_pos ) {
  Path* p_new_path = NULL; 

  RRTNode * p_st_first_node = NULL;
  RRTNode * p_gt_first_node = NULL;
  double st_delta_cost = 0.0;
  double gt_delta_cost = 0.0;
  _get_closest_node( via_pos, p_st_first_node, st_delta_cost, START_TREE_TYPE );
  _get_closest_node( via_pos, p_gt_first_node, gt_delta_cost, GOAL_TREE_TYPE );
   
  if ( _is_obstacle_free( p_st_first_node->m_pos, p_gt_first_node->m_pos ) ) {
    return p_new_path;
  }
   
  if( p_st_first_node != NULL && p_gt_first_node != NULL ) {
    Path* p_from_path = _get_subpath( p_st_first_node, START_TREE_TYPE );
    Path* p_to_path = _get_subpath( p_gt_first_node, GOAL_TREE_TYPE );

    p_new_path = _concatenate_paths( p_from_path, p_to_path ); 
  }

  return p_new_path;
}

void HARRTstar::_attach_new_node(RRTNode* p_node_new, RRTNode* p_nearest_node, std::list<RRTNode*> near_nodes) {
  double min_new_node_cost = p_nearest_node->m_cost + _calculate_cost(p_nearest_node->m_pos, p_node_new->m_pos);
  RRTNode* p_min_node = p_nearest_node;

  for(std::list<RRTNode*>::iterator it=near_nodes.begin();it!=near_nodes.end();it++) {
    RRTNode* p_near_node = *it;
    if ( true == _is_obstacle_free( p_near_node->m_pos, p_node_new->m_pos ) ) {
      double delta_cost = _calculate_cost( p_near_node->m_pos, p_node_new->m_pos );
      double new_cost = p_near_node->m_cost + delta_cost;
      if ( new_cost < min_new_node_cost ) {
        p_min_node = p_near_node;
        min_new_node_cost = new_cost;
      }
    }
  }

  bool added = _add_edge( p_min_node, p_node_new );
  if( added ) {
    p_node_new->m_cost = min_new_node_cost;
  }

}

void HARRTstar::_rewire_near_nodes(RRTNode* p_node_new, std::list<RRTNode*> near_nodes) {
  for( std::list<RRTNode*>::iterator it=near_nodes.begin(); it!=near_nodes.end(); it++ ) {
    RRTNode * p_near_node = (*it);

    if(p_near_node->m_pos ==p_node_new->m_pos ||  p_near_node->m_pos==_p_st_root->m_pos || p_node_new->mp_parent->m_pos==p_near_node->m_pos) {
      continue;
    }

    if( true == _is_obstacle_free( p_node_new->m_pos, p_near_node->m_pos ) ) {
      double temp_delta_cost = _calculate_cost( p_node_new->m_pos, p_near_node->m_pos );
      double temp_cost_from_new_node = p_node_new->m_cost + temp_delta_cost;
      if( temp_cost_from_new_node < p_near_node->m_cost ) {
        double min_delta_cost = p_near_node->m_cost - temp_cost_from_new_node;
        RRTNode * p_parent_node = p_near_node->mp_parent;
        bool removed = _remove_edge(p_parent_node, p_near_node);
        if(removed) {
          bool added = _add_edge(p_node_new, p_near_node);
          if( added ) {
            p_near_node->m_cost = temp_cost_from_new_node;
            _update_cost_to_children(p_near_node, min_delta_cost);
          }
        }
        else {
          std::cout << " Failed in removing " << std::endl;
        }
      }
    }
  }
}

void HARRTstar::_update_cost_to_children( RRTNode* p_node, double delta_cost ) {
  std::list<RRTNode*> child_list = _find_all_children( p_node );
  for( std::list<RRTNode*>::iterator it = child_list.begin(); it != child_list.end();it++ ) {
    RRTNode* p_child_node = (*it);
    if( p_child_node ) {
      p_child_node->m_cost -= delta_cost;
    }
  }
}

bool HARRTstar::_get_closest_node ( POS2D pos, RRTNode*& p_node_closet_to_goal, double& delta_cost, RRTree_type_t type ) {
  bool found = false;

  std::list<KDNode2D> near_nodes = _find_near( pos, type );
  double min_total_cost = std::numeric_limits<double>::max();

  for(std::list<KDNode2D>::iterator it=near_nodes.begin();
      it!=near_nodes.end();it++) {
    KDNode2D kd_node = (*it);
    RRTNode* p_node = kd_node.getRRTNode();
    double new_delta_cost = _calculate_cost(p_node->m_pos, _goal);
    double new_total_cost= p_node->m_cost + new_delta_cost;
    if (new_total_cost < min_total_cost) {
      min_total_cost = new_total_cost;
      p_node_closet_to_goal = p_node;
      delta_cost = new_delta_cost;
      found = true;
    }
  }
  return found;
}

void HARRTstar::dump_distribution(std::string filename) {
  std::ofstream myfile;
  myfile.open (filename.c_str());
  if(_pp_cost_distribution) {
    for(int i=0;i<_sampling_width;i++) {
      for(int j=0;j<_sampling_height;j++) {
        myfile << _pp_cost_distribution[i][j] << " ";
      }
      myfile << "\n";
    }
  }
  myfile.close();
}

Path* HARRTstar::_concatenate_paths( Path* p_from_path, Path* p_to_path ) {
  Path* p_new_path = new Path( p_from_path->m_start, p_to_path->m_start );
  Point2D from_path_end = Point2D( p_from_path->m_goal[0], p_from_path->m_goal[1] );
  Point2D to_path_end = Point2D( p_to_path->m_goal[0], p_to_path->m_goal[1] );
  std::vector< std::string > between_ids = _reference_frames->get_string( from_path_end, to_path_end , STRING_GRAMMAR_TYPE );
  double delta_cost = _calculate_cost( p_from_path->m_goal, p_to_path->m_goal );

  p_new_path->append_waypoints( p_from_path->m_way_points );
  p_new_path->append_substring( p_from_path->m_string );
  p_new_path->append_substring( between_ids );
  p_new_path->append_waypoints( p_to_path->m_way_points, true );
  p_new_path->append_substring( p_to_path->m_string, true );
  p_new_path->m_cost = p_from_path->m_cost + delta_cost + p_to_path->m_cost;
  
  return p_new_path;
}

Path* HARRTstar::_get_subpath( RRTNode* p_end_node, RRTree_type_t type ) {
  Path* p_subpath = NULL; 
  std::list<RRTNode*> node_list;
  get_parent_node_list( p_end_node , node_list );
  if( type == START_TREE_TYPE ) {
    p_subpath = new Path( _p_st_root->m_pos, p_end_node->m_pos );
  }
  else if ( type == GOAL_TREE_TYPE ) {
    p_subpath = new Path( _p_gt_root->m_pos, p_end_node->m_pos );
  }
  p_subpath->m_cost = p_end_node->m_cost;
  p_subpath->append_substring( p_end_node->m_substring ); 
  p_subpath->m_way_points.clear();
  for( std::list<RRTNode*>::iterator it = node_list.begin();
       it != node_list.end(); it ++ ) {
    RRTNode* p_rrt_node = (*it);
    p_subpath->m_way_points.push_back( p_rrt_node->m_pos ); 
  }
  return p_subpath;
}

std::vector<Path*> HARRTstar::get_paths() {
  std::vector<Path*> paths;
  if ( _p_string_class_mgr ) {
    paths = _p_string_class_mgr->export_paths();  
  } 
  return paths;
}

void HARRTstar::set_reference_frames( ReferenceFrameSet* p_reference_frames ) {
  _reference_frames = p_reference_frames;
}
