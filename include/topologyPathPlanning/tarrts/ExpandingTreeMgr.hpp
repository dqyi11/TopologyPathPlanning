#ifndef TOPOLOGYPATHPLANNING_TARRTS_EXPANDINGTREEMGR_HPP
#define TOPOLOGYPATHPLANNING_TARRTS_EXPANDINGTREEMGR_HPP

#include <vector>
#include "topologyPathPlanning/homotopy/ReferenceFrames.hpp"
#include "topologyPathPlanning/tarrts/KDTreeML2d.hpp"
#include "topologyPathPlanning/tarrts/ExpandingTree.hpp"

namespace topologyPathPlanning {

namespace tarrts {


  class SubRegionMgr {
  public:
    SubRegionMgr( homotopy::SubRegion* p_subregion );
    virtual ~SubRegionMgr();

    void addNode( ExpandingNode* p_node );
    ExpandingNode* findNode( std::string name);

    homotopy::SubRegion*        mpSubregion;
    std::vector<ExpandingNode*> mpNodes;
  };

  class LineSubSegmentMgr{
  public:
    LineSubSegmentMgr( homotopy::LineSubSegment* p_line_subsegment );
    virtual ~LineSubSegmentMgr();

    void addEdge( ExpandingEdge* p_edge );
    ExpandingEdge* findEdge( std::string name);

    homotopy::LineSubSegment*   mpLineSubsegment;
    std::vector<ExpandingEdge*> mpEdges;
  };
  
  class ExpandingTreeMgr {
  public:
    ExpandingTreeMgr();
    virtual ~ExpandingTreeMgr();

    void init( homotopy::StringGrammar* p_grammar, homotopy::ReferenceFrameSet* p_reference_frame_set );
    
    SubRegionMgr* findSubregionMgr( homotopy::SubRegion* p_subregion );
    LineSubSegmentMgr* findLineSubsegmentMgr( homotopy::LineSubSegment* p_line_subsegment );
    std::vector<StringClass*>& getStringClasses() { return mpStringClasses; };
    ExpandingTree* getExpandingTree() { return mpExpandingTree; };
    
    void exportSubregionMgrs( std::string filename );

    void dumpHistoricalData( std::string filename );
    void record();

  protected:
    ExpandingTree*                  mpExpandingTree;
    homotopy::StringGrammar*        mpStringGrammar;
    std::vector<StringClass*>       mpStringClasses;
    std::vector<SubRegionMgr*>      mpSubregionMgrs;
    std::vector<LineSubSegmentMgr*> mpLineSubsegmentMgrs;
  };

} // tarrts

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TARRTS_EXPANDINGTREEMGR_HPP
