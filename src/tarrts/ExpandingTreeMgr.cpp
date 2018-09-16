#include <fstream>
#include "topologyPathPlanning/tarrts/ExpandingTreeMgr.hpp"
#include "topologyPathPlanning/tarrts/MLRRTstar.hpp"

using namespace std;
using namespace topologyPathPlanning::homotopy;

namespace topologyPathPlanning {

namespace tarrts {

SubRegionMgr::SubRegionMgr( SubRegion* p_subregion ) {
  
  mpSubregion = p_subregion;
  mpNodes.clear();
}
    
SubRegionMgr::~SubRegionMgr() {
  
  mpSubregion = NULL;
  mpNodes.clear();
}

void SubRegionMgr::addNode( ExpandingNode* p_node ) {
  mpNodes.push_back( p_node );
}

ExpandingNode* SubRegionMgr::findNode( string name ) {
  for(vector<ExpandingNode*>::iterator it = mpNodes.begin();
      it != mpNodes.end(); it ++ ) {
    ExpandingNode* p_node = (*it);
    if( p_node->mName == name ) {
      return p_node;
    }
  }
  return NULL;
}

LineSubSegmentMgr::LineSubSegmentMgr( LineSubSegment* p_line_subsegment ) {

  mpLineSubsegment = p_line_subsegment;
  mpEdges.clear();
}

LineSubSegmentMgr::~LineSubSegmentMgr() {

  mpLineSubsegment = NULL;
  mpEdges.clear();
}

void LineSubSegmentMgr::addEdge( ExpandingEdge* p_edge ) {
  mpEdges.push_back( p_edge );
}

ExpandingEdge* LineSubSegmentMgr::findEdge( string name ) {
  for(vector<ExpandingEdge*>::iterator it = mpEdges.begin();
      it != mpEdges.end(); it ++ ) {
    ExpandingEdge* p_edge = (*it);
    if( p_edge->mName == name ) {
      return p_edge;
    }
  }
  return NULL;
}

ExpandingTreeMgr::ExpandingTreeMgr() {

  mpExpandingTree = NULL;
  mpStringGrammar = NULL;
}

ExpandingTreeMgr::~ExpandingTreeMgr() {
  
  mpExpandingTree = NULL;
  mpStringGrammar = NULL;
  mpStringClasses.clear();
  for(vector<SubRegionMgr*>::iterator it = mpSubregionMgrs.begin(); 
      it != mpSubregionMgrs.end(); it++) {
    SubRegionMgr* p_mgr = (*it);
    delete p_mgr;
    p_mgr = NULL;
  }
  for(vector<LineSubSegmentMgr*>::iterator it = mpLineSubsegmentMgrs.begin();
      it != mpLineSubsegmentMgrs.end(); it++) { 
    LineSubSegmentMgr* p_mgr = (*it);
    delete p_mgr;
    p_mgr = NULL;
  }
  mpSubregionMgrs.clear();
  mpLineSubsegmentMgrs.clear();
}

SubRegionMgr* ExpandingTreeMgr::findSubregionMgr( SubRegion* p_subregion ) {

  for(vector<SubRegionMgr*>::iterator it = mpSubregionMgrs.begin(); 
      it != mpSubregionMgrs.end(); it++) {
    SubRegionMgr* p_mgr = (*it);
    if( p_mgr->mpSubregion == p_subregion ) {
      return p_mgr;
    } 
  }
  return NULL;
}

LineSubSegmentMgr* ExpandingTreeMgr::findLineSubsegmentMgr( LineSubSegment* p_line_subsegment ) {

  for(vector<LineSubSegmentMgr*>::iterator it = mpLineSubsegmentMgrs.begin();
      it != mpLineSubsegmentMgrs.end(); it++) { 
    LineSubSegmentMgr* p_mgr = (*it);
    if( p_mgr->mpLineSubsegment == p_line_subsegment ) {
      return p_mgr;
    }
  }
  return NULL;
}

void ExpandingTreeMgr::init( StringGrammar* p_grammar, ReferenceFrameSet* p_reference_frame_set ) {
  if ( mpExpandingTree ) {
    delete mpExpandingTree;
    mpExpandingTree = NULL;
  }
  for( vector<StringClass*>::iterator it = mpStringClasses.begin();
       it != mpStringClasses.end(); it++ ) {
    StringClass* p_string_class = (*it);
    delete p_string_class;
    p_string_class = NULL;
  }
  mpStringClasses.clear();   

  mpStringGrammar = p_grammar;
  mpExpandingTree = new ExpandingTree();
  /* init string classes */
  mpStringClasses = mpExpandingTree->init( p_grammar, p_reference_frame_set );

  for( vector<ExpandingNode*>::iterator it = mpExpandingTree->mNodes.begin();
       it != mpExpandingTree->mNodes.end(); it ++ ) {
    ExpandingNode* p_node = (*it);
    SubRegionMgr* p_mgr = findSubregionMgr( p_node->mpSubregion );
    if( p_mgr == NULL ) {
      p_mgr = new SubRegionMgr( p_node->mpSubregion );
      mpSubregionMgrs.push_back( p_mgr );
    }
    p_mgr->addNode( p_node );   
  } 
  for( vector<ExpandingEdge*>::iterator it = mpExpandingTree->mEdges.begin(); 
       it != mpExpandingTree->mEdges.end(); it ++ ) {
    ExpandingEdge* p_edge = (*it);
    LineSubSegmentMgr* p_mgr = findLineSubsegmentMgr( p_edge->mpLineSubsegment );
    if( p_mgr == NULL ) {
      p_mgr = new LineSubSegmentMgr( p_edge->mpLineSubsegment );
      mpLineSubsegmentMgrs.push_back( p_mgr );
    }
    p_mgr->addEdge( p_edge );
  }
}

void ExpandingTreeMgr::exportSubregionMgrs( string filename ) {

  ofstream outfile( filename.c_str() );
  for( unsigned int i = 0; i < mpSubregionMgrs.size(); i ++ ) {
    SubRegionMgr* p_mgr = mpSubregionMgrs[i];
    if( p_mgr ) {
      outfile << p_mgr->mpSubregion->getName();
      outfile << " ( " << p_mgr->mpNodes.size() << " ) " << endl;
      for( unsigned int j = 0; j < p_mgr->mpNodes.size(); j ++ ) {
        ExpandingNode* p_node = p_mgr->mpNodes[j];
        outfile << "\t" << p_node << endl;
        if( p_node->mpInEdge ) {
          if( p_node->mpInEdge->mpFrom ) {
            outfile << "\t\t" << p_node->mpInEdge->mpFrom->mName << endl;
          }
        }
      }
    }
    outfile << endl;
  }
  outfile.close();
  return;
}

void ExpandingTreeMgr::record() {

  for(std::vector< StringClass* >::iterator it = mpStringClasses.begin();
      it != mpStringClasses.end(); it++ ) {
    StringClass* p_str_cls = (*it);
    if(p_str_cls) {
      p_str_cls->record();
    }
  }
}

void ExpandingTreeMgr::dumpHistoricalData( std::string filename ) {
  std::ofstream hist_data_file;
  hist_data_file.open(filename.c_str());
  for(std::vector< StringClass* >::iterator it = mpStringClasses.begin();
      it != mpStringClasses.end(); it++ ) {
    StringClass* p_str_cls = (*it);
    if(p_str_cls) {
      hist_data_file << p_str_cls->getName() << " : ";
      p_str_cls->writeHistoricalData( hist_data_file );
    }
  }
  hist_data_file.close();
}

} // tarrts

} // topologyPathPlanning
