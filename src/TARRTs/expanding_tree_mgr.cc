#include <fstream>
#include "tpp/tarrt/expanding_tree_mgr.h"
#include "tpp/tarrt/mlrrtstar.h"

using namespace std;
using namespace homotopy;
using namespace mlrrts;

SubRegionMgr::SubRegionMgr( SubRegion* p_subregion ) {
  
  mp_subregion = p_subregion;
  mp_nodes.clear();
}
    
SubRegionMgr::~SubRegionMgr() {
  
  mp_subregion = NULL;
  mp_nodes.clear();
}

void SubRegionMgr::add_node( ExpandingNode* p_node ) {
  mp_nodes.push_back( p_node );
}

ExpandingNode* SubRegionMgr::find_node( string name ) {
  for(vector<ExpandingNode*>::iterator it = mp_nodes.begin();
      it != mp_nodes.end(); it ++ ) {
    ExpandingNode* p_node = (*it);
    if( p_node->m_name == name ) {
      return p_node;
    }
  }
  return NULL;
}

LineSubSegmentMgr::LineSubSegmentMgr( LineSubSegment* p_line_subsegment ) {

  mp_line_subsegment = p_line_subsegment;
  mp_edges.clear();
}

LineSubSegmentMgr::~LineSubSegmentMgr() {

  mp_line_subsegment = NULL;
  mp_edges.clear();
}

void LineSubSegmentMgr::add_edge( ExpandingEdge* p_edge ) {
  mp_edges.push_back( p_edge );
}

ExpandingEdge* LineSubSegmentMgr::find_edge( string name ) {
  for(vector<ExpandingEdge*>::iterator it = mp_edges.begin();
      it != mp_edges.end(); it ++ ) {
    ExpandingEdge* p_edge = (*it);
    if( p_edge->m_name == name ) {
      return p_edge;
    }
  }
  return NULL;
}

ExpandingTreeMgr::ExpandingTreeMgr() {

  mp_expanding_tree = NULL;
  mp_string_grammar = NULL;
}

ExpandingTreeMgr::~ExpandingTreeMgr() {
  
  mp_expanding_tree = NULL;
  mp_string_grammar = NULL;
  mp_string_classes.clear();
  for(vector<SubRegionMgr*>::iterator it = mp_subregion_mgrs.begin(); 
      it != mp_subregion_mgrs.end(); it++) {
    SubRegionMgr* p_mgr = (*it);
    delete p_mgr;
    p_mgr = NULL;
  }
  for(vector<LineSubSegmentMgr*>::iterator it = mp_line_subsegment_mgrs.begin();
      it != mp_line_subsegment_mgrs.end(); it++) { 
    LineSubSegmentMgr* p_mgr = (*it);
    delete p_mgr;
    p_mgr = NULL;
  }
  mp_subregion_mgrs.clear();
  mp_line_subsegment_mgrs.clear();
}

SubRegionMgr* ExpandingTreeMgr::find_subregion_mgr( SubRegion* p_subregion ) {

  for(vector<SubRegionMgr*>::iterator it = mp_subregion_mgrs.begin(); 
      it != mp_subregion_mgrs.end(); it++) {
    SubRegionMgr* p_mgr = (*it);
    if( p_mgr->mp_subregion == p_subregion ) {
      return p_mgr;
    } 
  }
  return NULL;
}

LineSubSegmentMgr* ExpandingTreeMgr::find_line_subsegment_mgr( LineSubSegment* p_line_subsegment ) {

  for(vector<LineSubSegmentMgr*>::iterator it = mp_line_subsegment_mgrs.begin();
      it != mp_line_subsegment_mgrs.end(); it++) { 
    LineSubSegmentMgr* p_mgr = (*it);
    if( p_mgr->mp_line_subsegment == p_line_subsegment ) {
      return p_mgr;
    }
  }
  return NULL;
}

void ExpandingTreeMgr::init( StringGrammar* p_grammar, ReferenceFrameSet* p_reference_frame_set ) {
  if ( mp_expanding_tree ) {
    delete mp_expanding_tree;
    mp_expanding_tree = NULL;
  }
  for( vector<StringClass*>::iterator it = mp_string_classes.begin();
       it != mp_string_classes.end(); it++ ) {
    StringClass* p_string_class = (*it);
    delete p_string_class;
    p_string_class = NULL;
  }
  mp_string_classes.clear();   

  mp_string_grammar = p_grammar;
  mp_expanding_tree = new ExpandingTree();
  /* init string classes */
  mp_string_classes = mp_expanding_tree->init( p_grammar, p_reference_frame_set );

  for( vector<ExpandingNode*>::iterator it = mp_expanding_tree->m_nodes.begin();
       it != mp_expanding_tree->m_nodes.end(); it ++ ) {
    ExpandingNode* p_node = (*it);
    SubRegionMgr* p_mgr = find_subregion_mgr( p_node->mp_subregion );
    if( p_mgr == NULL ) {
      p_mgr = new SubRegionMgr( p_node->mp_subregion );
      mp_subregion_mgrs.push_back( p_mgr );
    }
    p_mgr->add_node( p_node );   
  } 
  for( vector<ExpandingEdge*>::iterator it = mp_expanding_tree->m_edges.begin(); 
       it != mp_expanding_tree->m_edges.end(); it ++ ) {
    ExpandingEdge* p_edge = (*it);
    LineSubSegmentMgr* p_mgr = find_line_subsegment_mgr( p_edge->mp_line_subsegment );
    if( p_mgr == NULL ) {
      p_mgr = new LineSubSegmentMgr( p_edge->mp_line_subsegment );
      mp_line_subsegment_mgrs.push_back( p_mgr );
    }
    p_mgr->add_edge( p_edge );
  }
}

void ExpandingTreeMgr::export_subregion_mgrs( string filename ) {

  ofstream outfile( filename.c_str() );
  for( unsigned int i = 0; i < mp_subregion_mgrs.size(); i ++ ) {
    SubRegionMgr* p_mgr = mp_subregion_mgrs[i];
    if( p_mgr ) {
      outfile << p_mgr->mp_subregion->get_name();
      outfile << " ( " << p_mgr->mp_nodes.size() << " ) " << endl;
      for( unsigned int j = 0; j < p_mgr->mp_nodes.size(); j ++ ) {
        ExpandingNode* p_node = p_mgr->mp_nodes[j];
        outfile << "\t" << p_node << endl;
        if( p_node->mp_in_edge ) {
          if( p_node->mp_in_edge->mp_from ) {
            outfile << "\t\t" << p_node->mp_in_edge->mp_from->m_name << endl;
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

  for(std::vector< StringClass* >::iterator it = mp_string_classes.begin();
      it != mp_string_classes.end(); it++ ) {
    StringClass* p_str_cls = (*it);
    if(p_str_cls) {
      p_str_cls->record();
    }
  }
}

void ExpandingTreeMgr::dump_historical_data( std::string filename ) {
  std::ofstream hist_data_file;
  hist_data_file.open(filename.c_str());
  for(std::vector< StringClass* >::iterator it = mp_string_classes.begin();
      it != mp_string_classes.end(); it++ ) {
    StringClass* p_str_cls = (*it);
    if(p_str_cls) {
      hist_data_file << p_str_cls->get_name() << " : ";
      p_str_cls->write_historical_data( hist_data_file );
    }
  }
  hist_data_file.close();
}
