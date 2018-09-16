#ifndef TOPOLOGYPATHPLANNING_TARRTS_EXPANDINGTREE_HPP
#define TOPOLOGYPATHPLANNING_TARRTS_EXPANDINGTREE_HPP

#include <vector>
#include <string>
#include <iostream>
#include "topologyPathPlanning/homotopy/StringGrammar.hpp"
#include "topologyPathPlanning/homotopy/ReferenceFrames.hpp"
#include "topologyPathPlanning/tarrts/KDTreeML2d.hpp"

namespace topologyPathPlanning {

namespace tarrts {

  class ExpandingEdge;
  class ExpandingNode;
  class Path;

  class StringClass {
  public:
    StringClass( std::vector< std::string > string );
    virtual ~StringClass();
    std::string getName();
    void addExpNode( ExpandingNode* p_node );
    void init( homotopy::ReferenceFrameSet* p_rf );
    void import( Path* p_path );
      
    std::vector< std::string >               m_string;
    KDTree2D*                                mp_kd_tree;
    std::vector< ExpandingNode* >            mp_exp_nodes;   
    std::vector< homotopy::ReferenceFrame* > mp_reference_frames; 
    std::vector< double >                    m_historical_data;

    void dumpHistoricalData( std::string filename );
    void writeHistoricalData( std::ostream& out );
    void record();

    Path*                         mp_path;  
    double                        m_cost;
    unsigned int                  m_created_iteration_num;
  };

  class ExpandingNode {
  public:
    ExpandingNode( std::string name );
    virtual ~ExpandingNode();

    ExpandingEdge* findOutEdge( std::string name );
    bool hasOutEdge( ExpandingEdge* p_edge );

    ExpandingNode* getParentNode();
    std::vector<ExpandingNode*> getChildNodes();
    void importAncestorSeq ( std::vector<ExpandingNode*> ancestor_seq );
    std::vector<std::string> getSubstring();

    POS2D sampleRandomPos();
    std::vector<POS2D> findFeasiblePath( ExpandingEdge* p_in_edge, ExpandingEdge* p_out_edge );
    bool isAncestor( ExpandingNode* p_node );
 
    std::string                 mName;
    ExpandingEdge*              mpInEdge;
    std::vector<ExpandingEdge*> mpOutEdges;
    homotopy::SubRegion*        mpSubregion;
    std::vector<StringClass*>   mpStringClasses;

    std::list<MLRRTNode*>       mpNodes;
    std::vector<ExpandingNode*> mpAncestorSeq;
  };
  
  class ExpandingEdge {
  public:
    ExpandingEdge( std::string name );
    virtual ~ExpandingEdge();
    
    void importAncestorSeq ( std::vector<ExpandingEdge*> ancestor_seq );
    std::vector<std::string> getSubstring();

    POS2D sampleRandomPos();

    std::string                 mName;
    ExpandingNode*              mpFrom;
    ExpandingNode*              mpTo;
    homotopy::LineSubSegment*   mpLineSubsegment;
    homotopy::ReferenceFrame*   mpReferenceFrame;
    POS2D                       mRandPos;
    std::vector<ExpandingEdge*> mpAncestorSeq;
  };

  class ExpandingTree {
  public:
    ExpandingTree();
    virtual ~ExpandingTree();

    std::vector< StringClass* > init( homotopy::StringGrammar* p_grammar, homotopy::ReferenceFrameSet* p_reference_frame_set = NULL );
    ExpandingNode* getRoot() { return mpRoot; }

    void output( std::string filename );

    int getIndex( ExpandingNode* p_node );
    std::vector<ExpandingNode*> getLeafNodes();

    void print();

    ExpandingNode* mpRoot;
    std::vector<ExpandingNode*> mNodes;
    std::vector<ExpandingEdge*> mEdges;
  };

} // tarrts

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TARRTS_EXPANDINGTREE_HPP
