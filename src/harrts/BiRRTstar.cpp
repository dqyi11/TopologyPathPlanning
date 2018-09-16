#include <limits>
#include <iostream>
#include <fstream>

#include "topologyPathPlanning/harrts/BiRRTstar.hpp"

#define OBSTACLE_THRESHOLD 200

using namespace topologyPathPlanning::homotopy;

namespace topologyPathPlanning {

namespace harrts {

BIRRTNode::BIRRTNode(POS2D pos) {
  mPos = pos;
  mCost = 0.0;
  mpParent = NULL;
  mChildNodes.clear();
  mSubstring.clear();
}

bool BIRRTNode::operator==(const BIRRTNode &other) {
  return mPos==other.mPos;
}

void BIRRTNode::clear_string() {
  mSubstring.clear();
}

void BIRRTNode::append_to_string( std::vector< std::string > ids ) {
  for( unsigned int i = 0; i < ids.size(); i ++ ) {
    std::string id = ids[i];
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

void Path::appendWaypoints( std::vector<POS2D> waypoints, bool reverse ) {
  if ( reverse ) {
    for( std::vector<POS2D>::reverse_iterator itr = waypoints.rbegin();
         itr != waypoints.rend(); itr++ ) {
      POS2D pos = (*itr);
      mWaypoints.push_back( pos );
    }
  }
  else {
    for( std::vector<POS2D>::iterator it = waypoints.begin();
         it != waypoints.end(); it++ ) {
      POS2D pos = (*it);
      mWaypoints.push_back( pos );
    }
  }
}

void Path::appendSubstring( std::vector< std::string > ids, bool reverse ) {
  if ( reverse ) {
    for( std::vector< std::string >::reverse_iterator itr = ids.rbegin();
         itr != ids.rend(); itr++ ) {
      std::string str = (*itr);
      mString.push_back( str );
    }
  }
  else {
    for( std::vector< std::string >::iterator it = ids.begin();
         it != ids.end(); it++ ) {
      std::string str = (*it);
      mString.push_back( str );
    }
  }
}

BIRRTstar::BIRRTstar( int width, int height, int segment_length ) {

  mSamplingWidth = width;
  mSamplingHeight = height;
  mSegmentLength = segment_length;
  mpSTRoot = NULL;
  mpGTRoot = NULL;
  mReferenceFrames = NULL;
  mRunType = RUN_BOTH_TREES_TYPE;

  mGrammarType = STRING_GRAMMAR_TYPE;
  mpSTKDTree = new KDTree2D( std::ptr_fun(tac) );
  mpGTKDTree = new KDTree2D( std::ptr_fun(tac) );

  mRange = (mSamplingWidth > mSamplingHeight) ? mSamplingWidth:mSamplingHeight;
  mSTBallRadius = mRange;
  mGTBallRadius = mRange;
  mObsCheckResolution = 1;
  mCurrentIteration = 0;
 
  mTheta = 10;

  mppCostDistribution = NULL;
  mpStringClassMgr = NULL;

  mppMapInfo = new int*[mSamplingWidth];
  for(int i=0;i<mSamplingWidth;i++) {
    mppMapInfo[i] = new int[mSamplingHeight];
    for(int j=0;j<mSamplingHeight;j++) {
      mppMapInfo[i][j] = 255;
    }
  }

  mSTNodes.clear();
  mGTNodes.clear();
}

BIRRTstar::~BIRRTstar() {
  if(mpSTKDTree) {
    delete mpSTKDTree;
    mpSTKDTree = NULL;
  }
  if(mpGTKDTree) {
    delete mpGTKDTree;
    mpGTKDTree = NULL;
  }

  if(mppMapInfo) {
    for(int i=0;i<mSamplingWidth;i++) {
      delete mppMapInfo[i];
      mppMapInfo[i] = NULL;
    }
    delete mppMapInfo;
    mppMapInfo = NULL;
  }
}

bool BIRRTstar::init( POS2D start, POS2D goal, COST_FUNC_PTR p_func, double** pp_cost_distribution, grammar_type_t grammar_type ) {
  if( mpSTRoot ) {
    delete mpSTRoot;
    mpSTRoot = NULL;
  }
  if( mpGTRoot ) {
    delete mpGTRoot;
    mpGTRoot = NULL;
  }
  if( mpStringClassMgr ) {
    delete mpStringClassMgr;
    mpStringClassMgr = NULL;
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
  
  std::cout << "Init grammar ... " << std::endl; 
  Point2D start_point( mStart[0], mStart[1] );
  Point2D goal_point( mGoal[0], mGoal[1] );
  setGrammarType(grammar_type);
  if( STRING_GRAMMAR_TYPE == grammar_type) {
    mStringGrammar = mReferenceFrames->getStringGrammar( start_point, goal_point );
  }
  else if( HOMOTOPIC_GRAMMAR_TYPE == grammar_type ) {
    mStringGrammar = mReferenceFrames->getHomotopicGrammar(start_point, goal_point );
  }
  std::cout << "Init String Class Mgr ... " << std::endl;
  mpStringClassMgr = new StringClassMgr( mStringGrammar );

  std::cout << "Init st_tree.." << std::endl;
  KDNode2D st_root( start );
  mpSTRoot = new BIRRTNode( start );
  mSTNodes.push_back(mpSTRoot);
  st_root.setBIRRTNode(mpSTRoot);
  mpSTKDTree->insert( st_root );
  
  std::cout << "Init gt_tree.." << std::endl;
  KDNode2D gt_root( goal );
  mpGTRoot = new BIRRTNode( goal );
  mGTNodes.push_back(mpGTRoot);
  gt_root.setBIRRTNode(mpGTRoot);
  mpGTKDTree->insert( gt_root );

  mCurrentIteration = 0;

}

void BIRRTstar::loadMap( int** pp_map ) {
  for(int i=0;i<mSamplingWidth;i++) {
    for(int j=0;j<mSamplingHeight;j++) {
      mppMapInfo[i][j] = pp_map[i][j];
    }
  }
}

POS2D BIRRTstar::sampling() {
  double x = rand();
  double y = rand();
  int int_x = x * ((double)(mSamplingWidth)/RAND_MAX);
  int int_y = y * ((double)(mSamplingHeight)/RAND_MAX);

  POS2D m(int_x,int_y);
  return m;
}

POS2D BIRRTstar::steer( POS2D pos_a, POS2D pos_b ) {
  POS2D new_pos( pos_a[0], pos_a[1] );
  double delta[2];
  delta[0] = pos_a[0] - pos_b[0];
  delta[1] = pos_a[1] - pos_b[1];
  double delta_len = sqrt(delta[0]*delta[0]+delta[1]*delta[1]);

  if (delta_len > mSegmentLength) {
    double scale = mSegmentLength / delta_len;
    delta[0] = delta[0] * scale;
    delta[1] = delta[1] * scale;

    new_pos.setX( pos_b[0]+delta[0] );
    new_pos.setY( pos_b[1]+delta[1] );
  }
  return new_pos;
}

bool BIRRTstar::isInObstacle( POS2D pos ) {
  int x = (int)pos[0];
  int y = (int)pos[1];
  if( mppMapInfo[x][y] < 255 ) {
    return true;
  }
  return false;
}


bool BIRRTstar::isObstacleFree( POS2D pos_a, POS2D pos_b ) {
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

void BIRRTstar::extend() {
  BIRRTNode* p_st_new_node = NULL;
  BIRRTNode* p_gt_new_node = NULL;
  Path* p_st_new_path = NULL;
  Path* p_gt_new_path = NULL;
  if( mRunType != RUN_GOAL_TREE_TYPE ) {
    p_st_new_node = extend(START_TREE_TYPE);
  }
  if( mRunType != RUN_START_TREE_TYPE ) {
    p_gt_new_node = extend(GOAL_TREE_TYPE);
  }
  
  if( mRunType != RUN_GOAL_TREE_TYPE ) {
    p_st_new_path = findPath( p_st_new_node->mPos );
  } 
  if( mRunType != RUN_START_TREE_TYPE ) {
    p_gt_new_path = findPath( p_gt_new_node->mPos ); 
  }

  if( mRunType != RUN_GOAL_TREE_TYPE ) {
    if( p_st_new_path ) {
      mpStringClassMgr->importPath( p_st_new_path, mCurrentIteration );
    }
  }
  if( mRunType != RUN_START_TREE_TYPE ) {
    if( p_gt_new_path ) {
      mpStringClassMgr->importPath( p_gt_new_path, mCurrentIteration );
    }
  }
  mpStringClassMgr->record();
  mCurrentIteration++;
}

BIRRTNode* BIRRTstar::extend( RRTree_type_t tree_type ) {
  bool node_inserted = false;
  while( false==node_inserted ) {
    POS2D rnd_pos = sampling();
    KDNode2D nearest_node = findNearest( rnd_pos, tree_type );

    if (rnd_pos[0]==nearest_node[0] && rnd_pos[1]==nearest_node[1]) {
      continue;
    }

    POS2D new_pos = steer( rnd_pos, nearest_node );
    if( true == contains(new_pos) ) {
      continue;
    }
    if( true == isInObstacle( new_pos ) ) {
      continue;
    }

    if( true == isObstacleFree( nearest_node, new_pos ) ) {
      if( true == isHomotopyEligible( nearest_node.getBIRRTNode(), new_pos, tree_type) ) {
        std::list<KDNode2D> near_list = findNear( new_pos, tree_type );
        KDNode2D new_node( new_pos );

        // create new node
        BIRRTNode * p_new_rnode = createNewNode( new_pos, tree_type );
        new_node.setBIRRTNode( p_new_rnode );
        if (tree_type == START_TREE_TYPE) {
          mpSTKDTree->insert( new_node );
        }
        else if(tree_type == GOAL_TREE_TYPE) {
          mpGTKDTree->insert( new_node );
        }
        node_inserted = true;

        BIRRTNode* p_nearest_rnode = nearest_node.getBIRRTNode();
        std::list<BIRRTNode*> near_rnodes;
        near_rnodes.clear();
        for( std::list<KDNode2D>::iterator itr = near_list.begin();
             itr != near_list.end(); itr++ ) {
          KDNode2D kd_node = (*itr);
          BIRRTNode* p_near_rnode = kd_node.getBIRRTNode();
          near_rnodes.push_back( p_near_rnode );
        }
        // attach new node to reference trees
        attachNewNode( p_new_rnode, p_nearest_rnode, near_rnodes, tree_type );
        // rewire near nodes of reference trees
        rewireNearNodes( p_new_rnode, near_rnodes, tree_type );

        return p_new_rnode;
      }
    }
  }
  return NULL;
}

KDNode2D BIRRTstar::findNearest( POS2D pos, RRTree_type_t tree_type ) {
  KDNode2D node( pos );
  if( START_TREE_TYPE == tree_type) {
    std::pair<KDTree2D::const_iterator,double> found = mpSTKDTree->find_nearest( node );
    KDNode2D near_node = *found.first;
    return near_node;
  }
  else if( GOAL_TREE_TYPE == tree_type ) {
    std::pair<KDTree2D::const_iterator,double> found = mpGTKDTree->find_nearest( node );
    KDNode2D near_node = *found.first;
    return near_node;
  }
  return node;
}

std::list<KDNode2D> BIRRTstar::findNear( POS2D pos, RRTree_type_t tree_type ) {
  std::list<KDNode2D> near_list;
  KDNode2D node(pos);

  int num_dimensions = 2;
  if ( START_TREE_TYPE == tree_type ) {
    int num_vertices = mpSTKDTree->size();
    mSTBallRadius =  mTheta * mRange * pow( log((double)(num_vertices + 1.0))/((double)(num_vertices + 1.0)), 1.0/((double)num_dimensions) );

    mpSTKDTree->find_within_range( node, mSTBallRadius, std::back_inserter( near_list ) );
  }
  else if ( GOAL_TREE_TYPE == tree_type ) {
    int num_vertices = mpGTKDTree->size();
    mGTBallRadius =  mTheta * mRange * pow( log((double)(num_vertices + 1.0))/((double)(num_vertices + 1.0)), 1.0/((double)num_dimensions) );

    mpGTKDTree->find_within_range( node, mGTBallRadius, std::back_inserter( near_list ) );
  }
  return near_list;
}


bool BIRRTstar::contains( POS2D pos ) {
  if(mpSTKDTree) {
    KDNode2D node( pos[0], pos[1] );
    KDTree2D::const_iterator it = mpSTKDTree->find(node);
    if( it!=mpSTKDTree->end() ) {
      return true;
    }
    else {
      return false;
    }
  }
  return false;
}

double BIRRTstar::calculateCost( POS2D& pos_a, POS2D& pos_b ) {
  return mpCostFunc(pos_a, pos_b, mppCostDistribution, this);
}

BIRRTNode* BIRRTstar::createNewNode(POS2D pos, RRTree_type_t tree_type) {
  BIRRTNode * pNode = new BIRRTNode(pos);
  if( tree_type == START_TREE_TYPE ) {
    mSTNodes.push_back(pNode);
  }
  else if( tree_type == GOAL_TREE_TYPE ) {
    mGTNodes.push_back(pNode);
  }
  return pNode;
}

bool BIRRTstar::removeEdge( BIRRTNode* p_node_parent, BIRRTNode*  p_node_child ) {
  if( p_node_parent==NULL ) {
    return false;
  }

  p_node_child->mpParent = NULL;
  bool removed = false;
  for( std::list<BIRRTNode*>::iterator it=p_node_parent->mChildNodes.begin();it!=p_node_parent->mChildNodes.end();it++ ) {
    BIRRTNode* p_current = (BIRRTNode*)(*it);
    if ( p_current == p_node_child || p_current->mPos==p_node_child->mPos ) {
      p_current->mpParent = NULL;
      p_current->clear_string();
      it = p_node_parent->mChildNodes.erase(it);
      removed = true;
    }
  }
  return removed;
}

bool BIRRTstar::hasEdge(BIRRTNode* p_node_parent, BIRRTNode* p_node_child) {
  if ( p_node_parent == NULL || p_node_child == NULL ) {
    return false;
  }
  for( std::list<BIRRTNode*>::iterator it=p_node_parent->mChildNodes.begin();it!=p_node_parent->mChildNodes.end();it++ ) {
    BIRRTNode* p_curr_node = (*it);
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

bool BIRRTstar::addEdge( BIRRTNode* p_node_parent, BIRRTNode* p_node_child ) {
  if( p_node_parent == NULL || p_node_child == NULL || p_node_parent == p_node_child ) {
    return false;
  }
  if ( p_node_parent->mPos == p_node_child->mPos ) {
    return false;
  }
  // generate the string of ID characters
  Point2D start( p_node_parent->mPos[0], p_node_parent->mPos[1] );
  Point2D goal( p_node_child->mPos[0], p_node_child->mPos[1] );
  //std::cout << "START " << start << " END " << goal << std::endl;
  std::vector< std::string > ids = mReferenceFrames->getString( start, goal, mGrammarType );
  p_node_child->clear_string();
  p_node_child->append_to_string( p_node_parent->mSubstring );
  p_node_child->append_to_string( ids );

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

std::list<BIRRTNode*> BIRRTstar::findAllChildren( BIRRTNode* p_node ) {
  int level = 0;
  bool finished = false;
  std::list<BIRRTNode*> child_list;

  std::list<BIRRTNode*> current_level_nodes;
  current_level_nodes.push_back( p_node );
  while( false==finished ) {
    std::list<BIRRTNode*> current_level_children;
    int child_list_num = child_list.size();

    for( std::list<BIRRTNode*>::iterator it=current_level_nodes.begin(); it!=current_level_nodes.end(); it++ ) {
      BIRRTNode* pCurrentNode = (*it);
      for( std::list<BIRRTNode*>::iterator itc=pCurrentNode->mChildNodes.begin(); itc!=pCurrentNode->mChildNodes.end();itc++ ) {
        BIRRTNode *p_child_node= (*itc);
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
      for( std::list<BIRRTNode*>::iterator itt=current_level_children.begin();itt!=current_level_children.end();itt++ ) {
        BIRRTNode * pTempNode = (*itt);
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


BIRRTNode* BIRRTstar::findAncestor(BIRRTNode* p_node) {
  return get_ancestor( p_node );
}

Path* BIRRTstar::findPath( POS2D via_pos ) {
  Path* p_new_path = NULL; 

  BIRRTNode * p_st_first_node = findNearest( via_pos, START_TREE_TYPE ).getBIRRTNode();
  BIRRTNode * p_gt_first_node = findNearest( via_pos, GOAL_TREE_TYPE ).getBIRRTNode();
   
  if ( false == isObstacleFree( p_st_first_node->mPos, p_gt_first_node->mPos ) ) {
    return p_new_path;
  }
   
  if( p_st_first_node != NULL && p_gt_first_node != NULL ) {
    Path* p_from_path = getSubpath( p_st_first_node, START_TREE_TYPE );
    Path* p_to_path = getSubpath( p_gt_first_node, GOAL_TREE_TYPE );

    p_new_path = concatenatePaths( p_from_path, p_to_path ); 
  }

  return p_new_path;
}

void BIRRTstar::attachNewNode(BIRRTNode* p_node_new, BIRRTNode* p_nearest_node, std::list<BIRRTNode*> near_nodes, RRTree_type_t tree_type) {
  double min_new_node_cost = p_nearest_node->mCost + calculateCost(p_nearest_node->mPos, p_node_new->mPos);
  BIRRTNode* p_min_node = p_nearest_node;

  for(std::list<BIRRTNode*>::iterator it=near_nodes.begin();it!=near_nodes.end();it++) {
    BIRRTNode* p_near_node = *it;
    if ( true == isObstacleFree( p_near_node->mPos, p_node_new->mPos ) ) {
      if ( true == isHomotopyEligible( p_near_node, p_node_new->mPos, tree_type ) ) {
        double delta_cost = calculateCost( p_near_node->mPos, p_node_new->mPos );
        double new_cost = p_near_node->mCost + delta_cost;
        if ( new_cost < min_new_node_cost ) {
          p_min_node = p_near_node;
          min_new_node_cost = new_cost;
        }
      }
    }
  }

  bool added = addEdge( p_min_node, p_node_new );
  if( added ) {
    p_node_new->mCost = min_new_node_cost;
  }

}

void BIRRTstar::rewireNearNodes(BIRRTNode* p_node_new, std::list<BIRRTNode*> near_nodes, RRTree_type_t tree_type) {
  for( std::list<BIRRTNode*>::iterator it=near_nodes.begin(); it!=near_nodes.end(); it++ ) {
    BIRRTNode * p_near_node = (*it);

    if(p_near_node->mPos ==p_node_new->mPos ||  p_near_node->mPos==mpSTRoot->mPos || p_node_new->mpParent->mPos==p_near_node->mPos) {
      continue;
    }

    if( true == isObstacleFree( p_node_new->mPos, p_near_node->mPos ) ) {
      if( true == isHomotopyEligible(p_near_node, p_node_new->mPos, tree_type) ) {
        double temp_delta_cost = calculateCost( p_node_new->mPos, p_near_node->mPos );
        double temp_cost_from_new_node = p_node_new->mCost + temp_delta_cost;
        if( temp_cost_from_new_node < p_near_node->mCost ) {
          double min_delta_cost = p_near_node->mCost - temp_cost_from_new_node;
          BIRRTNode * p_parent_node = p_near_node->mpParent;
          bool removed = removeEdge(p_parent_node, p_near_node);
          if(removed) {
            bool added = addEdge(p_node_new, p_near_node);
            if( added ) {
              p_near_node->mCost = temp_cost_from_new_node;
              updateCostToChildren(p_near_node, min_delta_cost);
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

void BIRRTstar::updateCostToChildren( BIRRTNode* p_node, double delta_cost ) {
  std::list<BIRRTNode*> child_list = findAllChildren( p_node );
  for( std::list<BIRRTNode*>::iterator it = child_list.begin(); it != child_list.end();it++ ) {
    BIRRTNode* p_child_node = (*it);
    if( p_child_node ) {
      p_child_node->mCost -= delta_cost;
    }
  }
}

bool BIRRTstar::getClosestNode ( POS2D pos, BIRRTNode*& p_node_closet_to_goal, double& delta_cost, RRTree_type_t tree_type ) {
  bool found = false;

  std::list<KDNode2D> near_nodes = findNear( pos, tree_type );
  double min_total_cost = std::numeric_limits<double>::max();

  for(std::list<KDNode2D>::iterator it=near_nodes.begin();
      it!=near_nodes.end();it++) {
    KDNode2D kd_node = (*it);
    BIRRTNode* p_node = kd_node.getBIRRTNode();
    double new_delta_cost = calculateCost(p_node->mPos, pos);
    double new_total_cost= p_node->mCost + new_delta_cost;
    if (new_total_cost < min_total_cost) {
      min_total_cost = new_total_cost;
      p_node_closet_to_goal = p_node;
      delta_cost = new_delta_cost;
      found = true;
    }
  }
  return found;
}

void BIRRTstar::dumpDistribution(std::string filename) {
  std::ofstream myfile;
  myfile.open (filename.c_str());
  if(mppCostDistribution) {
    for(int i=0;i<mSamplingWidth;i++) {
      for(int j=0;j<mSamplingHeight;j++) {
        myfile << mppCostDistribution[i][j] << " ";
      }
      myfile << "\n";
    }
  }
  myfile.close();
}

Path* BIRRTstar::concatenatePaths( Path* p_from_path, Path* p_to_path ) {
  Path* p_new_path = new Path( p_from_path->mStart, p_to_path->mStart );
  Point2D from_path_end( p_from_path->mGoal[0], p_from_path->mGoal[1] );
  Point2D to_path_end( p_to_path->mGoal[0], p_to_path->mGoal[1] );
  std::vector< std::string > between_ids = mReferenceFrames->getString( from_path_end, to_path_end , mGrammarType );
  double delta_cost = calculateCost( p_from_path->mGoal, p_to_path->mGoal );

  p_new_path->appendWaypoints( p_from_path->mWaypoints );
  p_new_path->appendSubstring( p_from_path->mString );
  p_new_path->appendSubstring( between_ids );
  p_new_path->appendWaypoints( p_to_path->mWaypoints, true );
  p_new_path->appendSubstring( p_to_path->mString, true );
  p_new_path->mCost = p_from_path->mCost + delta_cost + p_to_path->mCost;
  
  return p_new_path;
}

Path* BIRRTstar::getSubpath( BIRRTNode* p_end_node, RRTree_type_t tree_type ) {
  Path* p_subpath = NULL; 
  std::list<BIRRTNode*> node_list;
  get_parent_node_list( p_end_node , node_list );
  if( tree_type == START_TREE_TYPE ) {
    p_subpath = new Path( mpSTRoot->mPos, p_end_node->mPos );
  }
  else if ( tree_type == GOAL_TREE_TYPE ) {
    p_subpath = new Path( mpGTRoot->mPos, p_end_node->mPos );
  }
  p_subpath->mCost = p_end_node->mCost;
  p_subpath->appendSubstring( p_end_node->mSubstring ); 
  p_subpath->mWaypoints.clear();
  for( std::list<BIRRTNode*>::reverse_iterator itr = node_list.rbegin();
       itr != node_list.rend(); itr ++ ) {
    BIRRTNode* p_rrt_node = (*itr);
    p_subpath->mWaypoints.push_back( p_rrt_node->mPos ); 
  }
  return p_subpath;
}

std::vector<Path*> BIRRTstar::getPaths() {
  std::vector<Path*> paths;
  if ( mpStringClassMgr ) {
    paths = mpStringClassMgr->exportPaths();  
  } 
  return paths;
}

void BIRRTstar::setReferenceFrames( ReferenceFrameSet* p_reference_frames ) {
  mReferenceFrames = p_reference_frames;
}


bool BIRRTstar::isHomotopyEligible( BIRRTNode* p_node_parent, POS2D pos, RRTree_type_t tree_type ) {
  if( mReferenceFrames && mReferenceFrames->getStringConstraint().size()==0 ) {
      return true;
  }
  Point2D start( p_node_parent->mPos[0], p_node_parent->mPos[1] );
  Point2D end( pos[0], pos[1] );
  std::vector< std::string > ids = mReferenceFrames->getString( start, end, mGrammarType );
  std::vector< std::string > temp_ids;

  for( std::vector< std::string >::iterator it = p_node_parent->mSubstring.begin();
       it != p_node_parent->mSubstring.end(); it ++ ) {
    std::string id = (*it);
    temp_ids.push_back( id );
  }
  for( std::vector< std::string >::iterator it = ids.begin();
       it != ids.end(); it++) {
    std::string id = (*it);
    temp_ids.push_back( id );
  }

  if( tree_type == START_TREE_TYPE ) {
    return mReferenceFrames->isConstainedSubstring(temp_ids, false);
  } 
  else if( tree_type == GOAL_TREE_TYPE ) {
    return mReferenceFrames->isConstainedSubstring(temp_ids, true);
  } 
 
  return false;
}

} // harrts

} // topologyPathPlanning
