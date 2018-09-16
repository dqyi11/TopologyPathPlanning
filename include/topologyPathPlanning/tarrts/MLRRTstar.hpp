#ifndef TOPOLOGYPATHPLANNING_TARRTS_MLRRTSTAR_HPP
#define TOPOLOGYPATHPLANNING_TARRTS_MLRRTSTAR_HPP

#include <vector>
#include <list>
#include "topologyPathPlanning/homotopy/ReferenceFrames.hpp"
#include "topologyPathPlanning/tarrts/KDTreeML2d.hpp"
#include "topologyPathPlanning/tarrts/ExpandingTreeMgr.hpp"

namespace topologyPathPlanning {

namespace tarrts {

  typedef double (*COST_FUNC_PTR)(POS2D, POS2D, double**, void*);

  class MLRRTNode {
  
  public:
    MLRRTNode( POS2D pos );
    bool operator==( const MLRRTNode &other );
    void clearString();
    void appendToString( std::vector< std::string > ids );

    double                mCost;
    MLRRTNode*            mpParent;
    POS2D                 mPos;
    std::list<MLRRTNode*> mChildNodes;
    std::vector< std::string > mSubstring;

    ExpandingNode* mpMaster;
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

  class MLRRTstar {
  
  public:
    MLRRTstar( int width, int height, int segment_length );
    virtual ~MLRRTstar();
   
    bool init( POS2D start, POS2D goal, COST_FUNC_PTR p_func, double** pp_cost_distribution, homotopy::grammar_type_t grammar_type );
    void loadMap( int** pp_map );

    int getSamplingWidth() { return mSamplingWidth; }
    int getSamplingHeight() { return mSamplingHeight; }
    int getCurrentIteration() { return mCurrentIteration; }

 
    void extend();
    void updatePaths();
    std::vector<Path*> getPaths();
    Path* getPath( StringClass* p_string_class );
    Path* getPath( MLRRTNode* p_node );
    
    double calculateCost( POS2D& pos_a, POS2D& pos_b );
    int**& getMapInfo() { return mppMapInfo; }
    std::list<MLRRTNode*>& getNodes() { return mNodes; }

    void setReferenceFrames( homotopy::ReferenceFrameSet* p_reference_frames );
    homotopy::ReferenceFrameSet* getReferenceFrames() { return mReferenceFrames; }
    ExpandingTreeMgr* getExpandingTreeMgr() { return mpExpandingTreeMgr; }
    void setGrammarType( homotopy::grammar_type_t type ) { mGrammarType = type; }
    homotopy::grammar_type_t getGrammarType() { return mGrammarType; }

    bool inCurrentAndParentExpNode( POS2D pos, ExpandingNode* p_exp_node );

    bool isHomotopicEnforcement() { return mHomotopicEnforcement; }
    void setHomotopicEnforcement( bool enforcement ) { mHomotopicEnforcement = enforcement; }
  protected:
    POS2D sampling();
    POS2D steer( POS2D pos_a, POS2D pos_b );
    
    bool isObstacleFree( POS2D pos_a, POS2D pos_b );
    bool isInObstacle( POS2D pos );
    bool contains( POS2D pos );
    bool isHomotopicConstrained( MLRRTNode* p_node_parent, MLRRTNode* p_node_child );
    
    KDNode2D findNearest( POS2D pos, ExpandingNode* p_exp_node );
    std::list<KDNode2D> findNear( POS2D pos, ExpandingNode* p_exp_node );

    MLRRTNode* createNewNode( POS2D pos, ExpandingNode* p_exp_node );
    bool attachNewNode( MLRRTNode* p_node_new, std::list<MLRRTNode*> near_nodes );
    void rewireNearNodes( MLRRTNode* p_node_new, std::list<MLRRTNode*> near_nodes );
    void updateCostToChildren( MLRRTNode* p_node, double delta_cost );

    bool hasEdge( MLRRTNode* p_node_parent, MLRRTNode* p_node_child );
    bool addEdge( MLRRTNode* p_node_parent, MLRRTNode* p_node_child );
    bool removeEdge( MLRRTNode* p_node_parent, MLRRTNode* p_node_child );

    std::list<MLRRTNode*> findAllChildren( MLRRTNode* p_node );

    POS2D      mStart;
    POS2D      mGoal;
    MLRRTNode* mpRoot;
  
    homotopy::grammar_type_t mGrammarType;

    int mSamplingWidth;
    int mSamplingHeight;

    int**                        mppMapInfo;
    homotopy::ReferenceFrameSet* mReferenceFrames;
    homotopy::StringGrammar*     mpStringGrammar;
    ExpandingTreeMgr*            mpExpandingTreeMgr;
    KDTree2D*                    mpMasterKDTree;
    COST_FUNC_PTR                mpCostFunc;
    double**                     mppCostDistribution;
 
    std::list<MLRRTNode*>        mNodes;

    double mRange;
    double mSegmentLength;
    int    mObsCheckResolution;
    bool   mHomotopicEnforcement;

    double mTheta;
    int    mCurrentIteration;
  };

  inline MLRRTNode* getAncestor( MLRRTNode * node ) {
    if( NULL == node ) {
      return NULL;
    }
    if( NULL == node->mpParent ) {
      return node;
    }
    else {
      return getAncestor( node->mpParent );
    }
  }

  inline void getParentNodeList( MLRRTNode * node, std::list<MLRRTNode*>& path ) {
    if( node==NULL ) {
      return;
    }
    path.push_back( node );
    getParentNodeList( node->mpParent, path );
    return;
  }

} // tarrts

} // topologyPathPlanning


#endif // TOPOLOGYPATHPLANNING_TARRTS_MLRRTSTAR_HPP
