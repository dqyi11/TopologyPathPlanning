#include "topologyPathPlanning/tarrts/MLRRTstar.hpp"
#include "topologyPathPlanning/tarrts/MLUtil.hpp"

using namespace std;
using namespace topologyPathPlanning::homotopy;

namespace topologyPathPlanning {

namespace tarrts {

#define OBSTACLE_THRESHOLD 200

MLRRTNode::MLRRTNode(POS2D pos) {
  mPos = pos;
  mCost = 0.0;
  mpParent = NULL;
  mpMaster = NULL;
  mChildNodes.clear();
  mSubstring.clear();
}

bool MLRRTNode::operator==(const MLRRTNode &other) {
  if( mPos==other.mPos && mpParent == other.mpParent ) {
    return true;
  }
  return false;
}

void MLRRTNode::clearString() {
  mSubstring.clear();
}

void MLRRTNode::appendToString( vector< string > ids ) {
  for( unsigned int i = 0; i < ids.size(); i ++ ) {
    string id = ids[i];
    mSubstring.push_back( id );
  }
}

Path::Path(POS2D start, POS2D goal) {
  mStart = start;
  mGoal = goal;
  mCost = 0.0;

  mWaypoints.clear();
  mString.clear();
}

Path::~Path() {
  mCost = 0.0;
}

void Path::appendWaypoints( vector<POS2D> waypoints, bool reverse ) {
  if ( reverse ) {
    for( vector<POS2D>::reverse_iterator itr = waypoints.rbegin();
         itr != waypoints.rend(); itr++ ) {
      POS2D pos = (*itr);
      mWaypoints.push_back( pos );
    }
  }
  else {
    for( vector<POS2D>::iterator it = waypoints.begin();
         it != waypoints.end(); it++ ) {
      POS2D pos = (*it);
      mWaypoints.push_back( pos );
    }
  }
}

void Path::appendSubstring( vector< string > ids, bool reverse ) {
  if ( reverse ) {
    for( vector< string >::reverse_iterator itr = ids.rbegin();
         itr != ids.rend(); itr++ ) {
      string str = (*itr);
      mString.push_back( str );
    }
  }
  else {
    for( vector< string >::iterator it = ids.begin();
         it != ids.end(); it++ ) {
      string str = (*it);
      mString.push_back( str );
    }
  }
}

MLRRTstar::MLRRTstar( int width, int height, int segment_length ) {
  mSamplingWidth = width;
  mSamplingHeight = height;
  mSegmentLength = segment_length;

  mpRoot = NULL;
  mReferenceFrames = NULL;

  mGrammarType = STRING_GRAMMAR_TYPE;
  mpMasterKDTree = new KDTree2D( ptr_fun(tac) );

  mRange = (mSamplingWidth > mSamplingHeight) ? mSamplingWidth : mSamplingHeight;
  mObsCheckResolution = 1;
  mCurrentIteration = 0;
  mHomotopicEnforcement = false;

  mTheta = 10;
  mppCostDistribution = NULL;
  mpExpandingTreeMgr = NULL;

  mppMapInfo = new int*[mSamplingWidth];
  for(int i=0;i<mSamplingWidth;i++) {
    mppMapInfo[i] = new int[mSamplingHeight];
    for(int j=0;j<mSamplingHeight;j++) {
      mppMapInfo[i][j] = 255;
    }
  }
}

MLRRTstar::~MLRRTstar() {
  if( mpMasterKDTree ) {
    delete mpMasterKDTree;
    mpMasterKDTree = NULL;
  }
  if (mppMapInfo) {
    for(int i=0;i<mSamplingWidth;i++) {
      delete mppMapInfo[i];
      mppMapInfo[i] = NULL;
    }
    delete mppMapInfo;
    mppMapInfo = NULL;
  }
}

bool MLRRTstar::init( POS2D start, POS2D goal, COST_FUNC_PTR p_func, double** pp_cost_distribution, homotopy::grammar_type_t grammar_type ) {
  if( mpRoot ) {
    delete mpRoot;
    mpRoot = NULL;
  }
  if( mpExpandingTreeMgr ) {
    delete mpExpandingTreeMgr;
    mpExpandingTreeMgr = NULL;
  }
  if (mReferenceFrames == NULL) {
    return false;
  }

  mStart = start;
  mGoal = goal;
  mpCostFunc = p_func;

  if(pp_cost_distribution) {
    if(mppCostDistribution == NULL) {
      mppCostDistribution = new double*[mSamplingWidth];
      for(int i=0;i<mSamplingWidth;i++) {
        mppCostDistribution[i] = new double[mSamplingHeight];
      }
    }
    for(int i=0;i<mSamplingWidth;i++) {
      for(int j=0;j<mSamplingHeight;j++) {
        mppCostDistribution[i][j] = pp_cost_distribution[i][j];
      }
    }
  }

  cout << "Init grammar ... " << endl;
  Point2D start_point = toPoint2D( mStart );
  Point2D goal_point = toPoint2D( mGoal );
  setGrammarType(grammar_type);
  if( STRING_GRAMMAR_TYPE == grammar_type) {
    mpStringGrammar = mReferenceFrames->getStringGrammar( start_point, goal_point );
  }
  else if( HOMOTOPIC_GRAMMAR_TYPE == grammar_type ) {
    mpStringGrammar = mReferenceFrames->getHomotopicGrammar(start_point, goal_point );
  }
  cout << "Init String Class Mgr ... " << endl;
  mpExpandingTreeMgr = new ExpandingTreeMgr();
  mpExpandingTreeMgr->init( mpStringGrammar, mReferenceFrames );

  cout << "Init tree.." << endl;

  ExpandingNode* p_exp_node_root = mpExpandingTreeMgr->getExpandingTree()->getRoot();
  if( p_exp_node_root == NULL ) {
    cout << "NO EXP NODE ROOT" << endl;
    return false;
  }

  if( false == p_exp_node_root->mpSubregion->contains( toPoint2D( start ) ) ) {
    cout << "ROOT NODE REGION MISMATCH" << endl;
    return false;
  }

  mpRoot = createNewNode( start, p_exp_node_root );

  KDNode2D root( start );
  root.addMLRRTNode(mpRoot);
  mpMasterKDTree->insert( root );

  for( vector<StringClass*>::iterator it_str_cls = p_exp_node_root->mpStringClasses.begin();
       it_str_cls != p_exp_node_root->mpStringClasses.end(); it_str_cls++ ) {
    StringClass* p_str_cls = (*it_str_cls);
    if( p_str_cls ) {
      if( p_str_cls->mp_kd_tree ) {
        KDNode2D new_node( start );
        new_node.setPriMLRRTNode( mpRoot );
        p_str_cls->mp_kd_tree->insert( new_node );
      }
    }
  }

  mCurrentIteration = 0;

  return true;
}

void MLRRTstar::extend() {
  bool node_inserted = false;
  //int retry_cnt = 0;
  while( false == node_inserted ) {
    /*
    if( retry_cnt > 0 ) {
      cout << "RETRY " << retry_cnt << endl;
    }
    retry_cnt ++;
    */
    POS2D rnd_pos = sampling();
    KDNode2D nearest_node = findNearest( rnd_pos, NULL );

    if ( rnd_pos[0]==nearest_node[0] && rnd_pos[1]==nearest_node[1] ) {
      continue;
    }

    POS2D new_pos = steer( rnd_pos, nearest_node );
    if( true == contains( new_pos ) ) {
      continue;
    }
    if( true == isInObstacle( new_pos ) ) {
      continue;
    }

    if( true == isObstacleFree( nearest_node, new_pos ) ) {
       //cout << "NEW POS " << new_pos << endl;
       SubRegion* p_subregion = mReferenceFrames->getWorldMap()->inSubregion( toPoint2D( new_pos ) );
       if ( p_subregion ) {
         SubRegionMgr* p_mgr = mpExpandingTreeMgr->findSubregionMgr( p_subregion );
         if( p_mgr ) {
           KDNode2D new_master_node( new_pos );
           bool any_node_added = false;
           /* EACH EXPANDING NODE OF THE NEW POS */
           for( vector<ExpandingNode*>::iterator it = p_mgr->mpNodes.begin();
                it != p_mgr->mpNodes.end(); it ++ ) {
             ExpandingNode* p_exp_node = (*it);
             if( p_exp_node ) {
               // create new node
               MLRRTNode* p_new_rnode = createNewNode( new_pos, p_exp_node );
               KDNode2D nearest_node_in_class = findNearest( new_pos, p_exp_node );
               list<KDNode2D> near_list_in_class = findNear( new_pos, p_exp_node );

               //MLRRTNode* p_nearest_rnode = nearest_node_in_class.get_pri_mlrrtnode();
               list<MLRRTNode*> near_rnodes;
               near_rnodes.clear();
               for( list<KDNode2D>::iterator itr = near_list_in_class.begin();
                    itr != near_list_in_class.end(); itr ++ ) {
                 KDNode2D near_kd_node = (*itr);
                 MLRRTNode* p_near_rnode = near_kd_node.getPriMLRRTNode();
                 near_rnodes.push_back( p_near_rnode );
               }
               //std::cout << "IN " << p_exp_node->m_name << std::endl;
               // attach new noue
               if( attachNewNode( p_new_rnode, near_rnodes ) ) {
                 any_node_added = true;
                 new_master_node.addMLRRTNode( p_new_rnode );

                 if( p_exp_node ) {
                   for( vector<StringClass*>::iterator it_str_cls = p_exp_node->mpStringClasses.begin();
                        it_str_cls != p_exp_node->mpStringClasses.end(); it_str_cls++ ) {
                     StringClass* p_str_cls = (*it_str_cls);
                     if( p_str_cls->mp_kd_tree ) {
                       KDNode2D new_node( new_pos );
                       new_node.setPriMLRRTNode( p_new_rnode );
                       p_str_cls->mp_kd_tree->insert( new_node );
                     }
                   }
                   p_exp_node->mpNodes.push_back( p_new_rnode );
                 }
               }
               // rewire near nodes
               rewireNearNodes( p_new_rnode, near_rnodes );
            }
          }
          if ( any_node_added ) {
             mpMasterKDTree->insert( new_master_node );
             node_inserted = true;
          }
        }
      }
    }
  }
  if(mpExpandingTreeMgr) {
    mpExpandingTreeMgr->record();
  }

  mCurrentIteration ++;
}

POS2D MLRRTstar::sampling() {
  double x = rand();
  double y = rand();
  int int_x = x * ((double)(mSamplingWidth)/RAND_MAX);
  int int_y = y * ((double)(mSamplingHeight)/RAND_MAX);
  POS2D m(int_x, int_y);
  return m;
}

POS2D MLRRTstar::steer( POS2D pos_a, POS2D pos_b ) {
  POS2D new_pos( pos_a[0], pos_a[1] );
  double delta[2];
  delta[0] = pos_a[0] - pos_b[0];
  delta[1] = pos_a[1] - pos_b[1];
  double delta_len = sqrt(delta[0]*delta[0] + delta[1]*delta[1]);
  if ( delta_len > mSegmentLength ) {
    double scale = mSegmentLength / delta_len;
    delta[0] = delta[0] * scale;
    delta[1] = delta[1] * scale;

    new_pos.setX( pos_b[0]+delta[0] );
    new_pos.setY( pos_b[1]+delta[1] );
  }
  return new_pos;
}

bool MLRRTstar::isInObstacle( POS2D pos ) {
  int x = (int)pos[0];
  int y = (int)pos[1];
  if( mppMapInfo[x][y] < 255 ) {
    return true;
  }
  return false;
}

bool MLRRTstar::isObstacleFree( POS2D pos_a, POS2D pos_b ) {
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
      if ( y>=0 && y<mSamplingWidth && x>=0 && x<mSamplingHeight ) {
        if ( mppMapInfo[y][x] < OBSTACLE_THRESHOLD ) {
          return false;
        }
      }
    }
    else {
      if ( x>=0 && x<mSamplingWidth && y>=0 && y<mSamplingHeight ) {
        if ( mppMapInfo[x][y] < OBSTACLE_THRESHOLD ) {
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

KDNode2D MLRRTstar::findNearest( POS2D pos, ExpandingNode* p_exp_node ) {
  KDNode2D node( pos );
  KDNode2D nearest_node( pos );
  if( p_exp_node == NULL ) {
    pair<KDTree2D::const_iterator,double> found = mpMasterKDTree->find_nearest( node );
    nearest_node = *found.first;
  }
  else {
    /* find nearest in each string class */
    double nearest_distance = mSamplingWidth > mSamplingHeight ? mSamplingWidth : mSamplingHeight;

    for(vector<StringClass*>::iterator it = p_exp_node->mpStringClasses.begin(); it != p_exp_node->mpStringClasses.end(); it ++ ) {
      StringClass* p_class = (*it);
      pair<KDTree2D::const_iterator,double> found = p_class->mp_kd_tree->find_nearest( node );
      KDNode2D nearest_node_in_class = *found.first;
      double distance_in_class = found.second;

      if( (distance_in_class < nearest_distance) &&
           inCurrentAndParentExpNode( nearest_node, p_exp_node ) ) {
        nearest_distance = distance_in_class;
        nearest_node = nearest_node_in_class;
      }
    }

  }
  return nearest_node;
}

list<KDNode2D> MLRRTstar::findNear( POS2D pos, ExpandingNode* p_exp_node ) {
  list<KDNode2D> near_list;
  KDNode2D node(pos);
  int num_dimensions = 2;
  if( p_exp_node == NULL ) {
    int num_vertices = mpMasterKDTree->size();
    double ball_radius =  mTheta * mRange * pow( log((double)(num_vertices + 1.0))/((double)(num_vertices + 1.0)), 1.0/((double)num_dimensions) );

    mpMasterKDTree->find_within_range( node, ball_radius, back_inserter( near_list ) );
  }
  else {

    for(vector<StringClass*>::iterator it = p_exp_node->mpStringClasses.begin(); it != p_exp_node->mpStringClasses.end(); it ++ ) {
      StringClass* p_class = (*it);
      list<KDNode2D> near_list_in_class;
      int num_vertices = p_class->mp_kd_tree->size();
      double ball_radius =  mTheta * mRange * pow( log((double)(num_vertices + 1.0))/((double)(num_vertices + 1.0)), 1.0/((double)num_dimensions) );
      p_class->mp_kd_tree->find_within_range( node, ball_radius, back_inserter( near_list_in_class ) );
      //cout << "NEAR LIST IN CLASS " << near_list_in_class.size() << endl;
      for( list<KDNode2D>::iterator it_cls = near_list_in_class.begin();
           it_cls != near_list_in_class.end(); it_cls ++ ) {
        KDNode2D kdnode = (*it_cls);
        if ( inCurrentAndParentExpNode( kdnode, p_exp_node ) ) {
          near_list.push_back( kdnode );
        }
      }
    }
  }
  //cout << "NEAR LIST " << near_list.size() << endl;
  return near_list;
}

bool MLRRTstar::inCurrentAndParentExpNode( POS2D pos, ExpandingNode* p_exp_node ) {
  if( p_exp_node ) {
    if( p_exp_node->mpSubregion ) {
      if( p_exp_node->mpSubregion->contains( toPoint2D( pos ) ) ) {
        return true;
      }
    }
    if( p_exp_node->mpInEdge ){
      if( p_exp_node->mpInEdge->mpFrom ){
        if( p_exp_node->mpInEdge->mpFrom->mpSubregion ) {
          if( p_exp_node->mpInEdge->mpFrom->mpSubregion->contains( toPoint2D( pos ) ) ) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

bool MLRRTstar::contains( POS2D pos ) {
  if(mpMasterKDTree) {
    KDNode2D node( pos[0], pos[1] );
    KDTree2D::const_iterator it = mpMasterKDTree->find(node);
    if( it!=mpMasterKDTree->end() ) {
      return true;
    }
    else {
      return false;
    }
  }
  return false;
}

MLRRTNode* MLRRTstar::createNewNode( POS2D pos, ExpandingNode* p_exp_node ) {
  MLRRTNode* p_node = new MLRRTNode( pos );
  mNodes.push_back( p_node );
  if( p_exp_node ) {
    p_exp_node->mpNodes.push_back( p_node );
  }
  p_node->mpMaster = p_exp_node;
  return p_node;
}

void MLRRTstar::setReferenceFrames( ReferenceFrameSet* p_reference_frames ) {
  mReferenceFrames = p_reference_frames;
}

void MLRRTstar::updatePaths() {

  if( mpExpandingTreeMgr ) {
    vector<StringClass*> string_classes = mpExpandingTreeMgr->getStringClasses();
    for( vector<StringClass*>::iterator it = string_classes.begin();
         it != string_classes.end(); it ++ ) {
      StringClass* p_string_class = (*it);
      Path* p_path = getPath( p_string_class );
      p_string_class->import( p_path );
      /*
      p_string_class->mp_path = p_path;
      if(p_path) {
        p_string_class->m_cost = p_path->m_cost;
      }*/
    }
  }
}

vector<Path*> MLRRTstar::getPaths() {
  vector<Path*> paths;
  updatePaths();
  if( mpExpandingTreeMgr ) {
    vector<StringClass*> string_classes = mpExpandingTreeMgr->getStringClasses();
    for( vector<StringClass*>::iterator it = string_classes.begin();
         it != string_classes.end(); it ++ ) {
      StringClass* p_string_class = (*it);
      if(p_string_class) {
        paths.push_back( p_string_class->mp_path );
      }
    }
  }
  return paths;
}

Path* MLRRTstar::getPath( StringClass* p_string_class ) {

  if( p_string_class ) {
    int exp_node_num = p_string_class->mp_exp_nodes.size();
    ExpandingNode* p_last_exp_node = p_string_class->mp_exp_nodes[exp_node_num-1];
    if( p_last_exp_node ) {
      // get a set of nodes, sort them by distance
      // find the first one belong to the exp node
      std::list<KDNode2D> near_kdnodes = findNear( mGoal, p_last_exp_node );
      for(std::list<KDNode2D>::iterator it = near_kdnodes.begin();
          it != near_kdnodes.end();it++) {
        KDNode2D kdnode = (*it);
        MLRRTNode* p_near_goal = kdnode.getPriMLRRTNode();
        if(p_last_exp_node==p_near_goal->mpMaster) {
          if( p_near_goal ) {
            if( isObstacleFree( p_near_goal->mPos, mGoal ) == true ) {
              Path* p_path = getPath( p_near_goal );
              return p_path;
            }
          }
        }
      }
    }
  }
  return NULL;
}

Path* MLRRTstar::getPath( MLRRTNode* p_node ) {
  list<MLRRTNode*> node_list;
  getParentNodeList( p_node, node_list );
  Path* p_path = new Path( mStart, mGoal );
  for( list<MLRRTNode*>::reverse_iterator itr = node_list.rbegin();
       itr != node_list.rend(); itr ++ ) {
    MLRRTNode* p_rrt_node = (*itr);
    p_path->mWaypoints.push_back( p_rrt_node->mPos );
  }
  p_path->mWaypoints.push_back( mGoal );
  p_path->appendSubstring( p_node->mSubstring );
  p_path->mCost = p_node->mCost + calculateCost( p_node->mPos, mGoal );
  return p_path;
}


bool MLRRTstar::attachNewNode( MLRRTNode* p_node_new, list<MLRRTNode*> near_nodes ) {
  double min_new_node_cost = -1;
  MLRRTNode* p_min_node = NULL;

  for(list<MLRRTNode*>::iterator it = near_nodes.begin(); it != near_nodes.end(); it++) {
    MLRRTNode* p_near_node = (*it);
    if( true == isObstacleFree( p_near_node->mPos, p_node_new->mPos ) ) {
      bool eligible = true;
      if( mHomotopicEnforcement ) {
        eligible = isHomotopicConstrained( p_near_node, p_node_new );
      }
      if( eligible ) {
        double delta_cost = calculateCost( p_near_node->mPos, p_node_new->mPos );
        double new_cost = p_near_node->mCost + delta_cost;
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

  bool added = addEdge( p_min_node, p_node_new );
  if( added ) {
    p_node_new->mCost = min_new_node_cost;
    return true;
  }
  return false;
}

void MLRRTstar::rewireNearNodes( MLRRTNode* p_node_new, list<MLRRTNode*> near_nodes ) {
  for( list<MLRRTNode*>::iterator it = near_nodes.begin();
       it != near_nodes.end(); it++ ) {
    MLRRTNode* p_near_node = (*it);

    if( p_near_node->mPos == p_node_new->mPos ||
        p_near_node->mPos == mpRoot->mPos ) {
      continue;
    }

    if( true == isObstacleFree( p_node_new->mPos, p_near_node->mPos ) ) {
      bool eligible = true;
      if( mHomotopicEnforcement ) {
        eligible = isHomotopicConstrained( p_node_new, p_near_node );
      }
      if( eligible ) {
        double temp_delta_cost = calculateCost( p_node_new->mPos, p_near_node->mPos );
        double temp_cost_from_new_node = p_node_new->mCost + temp_delta_cost;
        if( temp_cost_from_new_node < p_near_node->mCost ) {
          double min_delta_cost = p_near_node->mCost - temp_cost_from_new_node;
          MLRRTNode* p_parent_node = p_near_node->mpParent;
          bool removed = removeEdge( p_parent_node, p_near_node );
          if( removed ) {
            bool added = addEdge( p_node_new, p_near_node );
            if( added ) {
              p_near_node->mCost = temp_cost_from_new_node;
              updateCostToChildren( p_near_node, min_delta_cost );
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

double MLRRTstar::calculateCost( POS2D& pos_a, POS2D& pos_b ) {
  return mpCostFunc( pos_a, pos_b, mppCostDistribution, this);
}

bool MLRRTstar::hasEdge( MLRRTNode* p_node_parent, MLRRTNode* p_node_child ) {
  if ( p_node_parent == NULL || p_node_child == NULL ) {
    return false;
  }
  for( list<MLRRTNode*>::iterator it=p_node_parent->mChildNodes.begin();it!=p_node_parent->mChildNodes.end();it++ ) {
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

bool MLRRTstar::addEdge( MLRRTNode* p_node_parent, MLRRTNode* p_node_child ) {
  if( p_node_parent == NULL || p_node_child == NULL || p_node_parent == p_node_child ) {
    return false;
  }
  if ( p_node_parent->mPos == p_node_child->mPos ) {
    return false;
  }
  // generate the string of ID characters
  Point2D start = toPoint2D( p_node_parent->mPos );
  Point2D goal = toPoint2D( p_node_child->mPos );
  //std::cout << "START " << start << " END " << goal << std::endl;
  vector< string > ids = mReferenceFrames->getString( start, goal, mGrammarType );
  p_node_child->clearString();
  p_node_child->appendToString( p_node_parent->mSubstring );
  p_node_child->appendToString( ids );

  if ( true == hasEdge( p_node_parent, p_node_child ) ) {
    p_node_child->mpParent = p_node_parent;
  }
  else {
    p_node_parent->mChildNodes.push_back( p_node_child );
    p_node_child->mpParent = p_node_parent;
  }
  p_node_child->mChildNodes.unique();

  return true;
}

bool MLRRTstar::removeEdge( MLRRTNode* p_node_parent, MLRRTNode* p_node_child ) {
  if( p_node_parent==NULL ) {
    return false;
  }

  p_node_child->mpParent = NULL;
  bool removed = false;
  for( list<MLRRTNode*>::iterator it=p_node_parent->mChildNodes.begin();it!=p_node_parent->mChildNodes.end();it++ ) {
    MLRRTNode* p_current = (MLRRTNode*)(*it);
    if ( p_current == p_node_child || p_current->mPos==p_node_child->mPos ) {
      p_current->mpParent = NULL;
      p_current->clearString();
      it = p_node_parent->mChildNodes.erase(it);
      removed = true;
    }
  }
  return removed;
}

bool MLRRTstar::isHomotopicConstrained( MLRRTNode* p_node_parent, MLRRTNode* p_node_child ) {
  if( p_node_parent && p_node_child ) {
    if( p_node_parent->mpMaster && p_node_child->mpMaster ) {
      ExpandingNode* p_exp_node_parent = p_node_parent->mpMaster;
      ExpandingNode* p_exp_node_child = p_node_child->mpMaster;
      if( p_exp_node_parent == p_exp_node_child ) {
        return true;
      }

      if( p_exp_node_child->mpInEdge ) {
        //cout << "Parent " << p_exp_node_parent->m_name <<  endl;
        //cout << "Child " << p_exp_node_child->m_name << " ( " << p_exp_node_child->mp_in_edge->mp_from->m_name << " ) " << endl;
        //cout << "COMPARE child=" << p_exp_node_child->m_name << "(" << p_exp_node_child->mp_in_edge->mp_from->m_name << ") " << p_exp_node_parent->m_name << endl;
        if( p_exp_node_child->mpInEdge->mpFrom == p_exp_node_parent ) {
          //cout << "HAPPENED " << endl;
          if( p_exp_node_child->mpInEdge->mpReferenceFrame->isLineCrossed( toPoint2D( p_node_parent->mPos ), toPoint2D( p_node_child->mPos ) ) ) {
            return true;
          }
        }
      }
    }
    else{
      cout << "NULL PARENT" << endl;
    }
  }

  return false;
}

void MLRRTstar::updateCostToChildren( MLRRTNode* p_node, double delta_cost ) {
  list<MLRRTNode*> child_list = findAllChildren( p_node );
  for( list<MLRRTNode*>::iterator it = child_list.begin();
       it != child_list.end(); it++ ) {
    MLRRTNode* p_child_node = (*it);
    if( p_child_node ) {
      p_child_node->mCost -= delta_cost;
    }
  }
}

list<MLRRTNode*> MLRRTstar::findAllChildren( MLRRTNode* p_node ) {
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
      for( list<MLRRTNode*>::iterator itc = p_current_node->mChildNodes.begin();
           itc != p_current_node->mChildNodes.end(); itc ++ ) {
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
    else if( (signed)child_list.size() == child_list_num ) {
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

} // tarrts

} // topologyPathPlanning
