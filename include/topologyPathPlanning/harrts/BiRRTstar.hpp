#ifndef TOPOLOGYPATHPLANNING_HARRTS_BIRRTSTAR_HPP
#define TOPOLOGYPATHPLANNING_HARRTS_BIRRTSTAR_HPP

#include <vector>
#include <list>
#include "topologyPathPlanning/homotopy/ReferenceFrames.hpp"
#include "topologyPathPlanning/harrts/KDTree2d.hpp"
#include "topologyPathPlanning/harrts/StringClassMgr.hpp"

namespace topologyPathPlanning {

namespace harrts {

  typedef double (*COST_FUNC_PTR)(POS2D, POS2D, double**, void*);

  typedef enum{
    START_TREE_TYPE,
    GOAL_TREE_TYPE
  } RRTree_type_t;

  typedef enum{
    RUN_START_TREE_TYPE,
    RUN_GOAL_TREE_TYPE,
    RUN_BOTH_TREES_TYPE
  } RRTree_run_type_t;

  class BIRRTNode {

  public:
    BIRRTNode( POS2D pos );
    bool operator==( const BIRRTNode &other );
    void clear_string();
    void append_to_string( std::vector< std::string > ids );

    double   mCost;
    BIRRTNode* mpParent;
    POS2D    mPos;
    std::list<BIRRTNode*> mChildNodes;
    std::vector< std::string > mSubstring;
  };

  class Path {

  public:
    Path(POS2D start, POS2D goal);
    ~Path();
    void appendWaypoints( std::vector<POS2D> waypoints, bool reverse = false );
    void appendSubstring( std::vector< std::string > ids, bool reverse = false );

    double mCost;
    POS2D  mStart;
    POS2D  mGoal;
    std::vector<POS2D> mWaypoints;
    std::vector< std::string > mString;
  };

  class BIRRTstar {

  public:
    BIRRTstar(int width, int height, int segment_length);
    virtual ~BIRRTstar();

    bool init( POS2D start, POS2D goal, COST_FUNC_PTR p_func, double** pp_cost_distrinution, homotopy::grammar_type_t grammar_type = homotopy::STRING_GRAMMAR_TYPE );
    void loadMap( int** pp_map );

    int getSamplingWidth() { return mSamplingWidth; }
    int getSamplingHeight() { return mSamplingHeight; }
    int getCurrentIteration() { return mCurrentIteration; }

    std::list<BIRRTNode*>& getSTNodes() { return mSTNodes; }
    std::list<BIRRTNode*>& getGTNodes() { return mGTNodes; }

    int**& getMapInfo() { return mppMapInfo; }
    double getSTBallRadius() { return mSTBallRadius; }
    double getGTBallRadius() { return mGTBallRadius; }

    void extend();
    Path* findPath( POS2D via_pos );
    std::vector<Path*> getPaths();

    void setReferenceFrames( homotopy::ReferenceFrameSet* p_reference_frames );
    homotopy::ReferenceFrameSet* getReferenceFrames() { return mReferenceFrames; }
    StringClassMgr* getStringClassMgr() { return mpStringClassMgr; }

    void setRunType( RRTree_run_type_t tree_type ) { mRunType = tree_type; }
    RRTree_run_type_t getRunType() { return mRunType; }
    homotopy::grammar_type_t getGrammarType() { return mGrammarType; }
    void dumpDistribution(std::string filename);

  protected:
    POS2D sampling();
    POS2D steer( POS2D pos_a, POS2D pos_b );
    BIRRTNode* extend(RRTree_type_t tree_type);

    Path* concatenatePaths( Path* p_from_path, Path* p_to_path );
    Path* getSubpath( BIRRTNode* p_end_node, RRTree_type_t tree_type );

    KDNode2D findNearest( POS2D pos, RRTree_type_t tree_type );
    std::list<KDNode2D> findNear( POS2D pos, RRTree_type_t tree_type );

    bool isHomotopyEligible( BIRRTNode* p_node_parent, POS2D pos, RRTree_type_t tree_type );
    bool isObstacleFree( POS2D pos_a, POS2D pos_b );
    bool isInObstacle( POS2D pos );
    bool contains( POS2D pos );

    double calculateCost( POS2D& pos_a, POS2D& pos_b );

    BIRRTNode* createNewNode( POS2D pos, RRTree_type_t tree_type );
    bool removeEdge( BIRRTNode* p_node_parent, BIRRTNode* p_node_child );
    bool hasEdge( BIRRTNode* p_node_parent, BIRRTNode* p_node_child );
    bool addEdge( BIRRTNode* p_node_parent, BIRRTNode* p_node_child );

    std::list<BIRRTNode*> findAllChildren( BIRRTNode* node );

    void attachNewNode( BIRRTNode* p_node_new, BIRRTNode* p_nearest_node, std::list<BIRRTNode*> near_nodes, RRTree_type_t type );
    void rewireNearNodes( BIRRTNode* p_node_new, std::list<BIRRTNode*> near_nodes, RRTree_type_t tree_type );
    void updateCostToChildren( BIRRTNode* p_node, double delta_cost );
    bool getClosestNode( POS2D pos, BIRRTNode*& p_node_closest, double& delta_cost, RRTree_type_t tree_type );

    BIRRTNode* findAncestor( BIRRTNode* p_node );
    void setGrammarType( homotopy::grammar_type_t grammar_type ) { mGrammarType = grammar_type; }

    homotopy::ReferenceFrameSet* mReferenceFrames;
    homotopy::StringGrammar*     mStringGrammar;
 
    POS2D    mStart;
    POS2D    mGoal;
    BIRRTNode* mpSTRoot;
    BIRRTNode* mpGTRoot;

    homotopy::grammar_type_t mGrammarType;

    BIRRTNode* mpSTNewNode;
    BIRRTNode* mpGTNewNode;
    BIRRTNode* mpSTConnectedNode;
    BIRRTNode* mpGTConnectedNode;

    int mSamplingWidth;
    int mSamplingHeight;

    int**           mppMapInfo;
    StringClassMgr* mpStringClassMgr;
    KDTree2D*       mpSTKDTree;
    KDTree2D*       mpGTKDTree;
    COST_FUNC_PTR   mpCostFunc;
    double**        mppCostDistribution;

    std::list<BIRRTNode*> mSTNodes;
    std::list<BIRRTNode*> mGTNodes;

    RRTree_run_type_t mRunType;
    double mRange;
    double mSTBallRadius;
    double mGTBallRadius;
    double mSegmentLength;
    int    mObsCheckResolution;

    double mTheta;
    int    mCurrentIteration;
  };

  inline BIRRTNode* get_ancestor( BIRRTNode * node ) {
    if( NULL == node ) {
      return NULL;
    }
    if( NULL == node->mpParent ) {
      return node;
    }
    else {
      return get_ancestor( node->mpParent );
    }
  }

  inline void get_parent_node_list( BIRRTNode * node, std::list<BIRRTNode*>& path ) {
    if( node==NULL ) {
      return;
    }
    path.push_back( node );
    get_parent_node_list( node->mpParent, path );
    return;
  }

} // harrts

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_HARRTS_BIRRTSTAR_HPP
