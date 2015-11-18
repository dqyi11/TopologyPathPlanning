#include "expanding_tree_mgr.h"
#include "mlrrtstar.h"

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

ExpandingNode* SubRegionMgr::find_node( std::string name ) {
  for(std::vector<ExpandingNode*>::iterator it = mp_nodes.begin();
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

ExpandingEdge* LineSubSegmentMgr::find_edge( std::string name ) {
  for(std::vector<ExpandingEdge*>::iterator it = mp_edges.begin();
      it != mp_edges.end(); it ++ ) {
    ExpandingEdge* p_edge = (*it);
    if( p_edge->m_name == name ) {
      return p_edge;
    }
  }
  return NULL;
}

StringClass::StringClass( std::vector< std::string > string ) {
  m_string = string;
  m_cost = 0.0;
  mp_path = NULL;
}

StringClass::~StringClass() {
  m_string.clear();
  m_cost = 0.0;
  mp_path = NULL;
}

std::string StringClass::get_name() {
  std::string name = "";
  for( unsigned int i = 0; i < m_string.size(); i ++ ) {
    if (i > 0) { 
      name += " ";
    }
    name += m_string[i];
  }
  return name;
}

ExpandingTreeMgr::ExpandingTreeMgr() {

  mp_expanding_tree = NULL;
  mp_string_grammar = NULL;
}

ExpandingTreeMgr::~ExpandingTreeMgr() {
  
  mp_expanding_tree = NULL;
  mp_string_grammar = NULL;
  m_string_classes.clear();
}

void ExpandingTreeMgr::init( StringGrammar* p_grammar, WorldMap* p_worldmap ) {
  if ( mp_expanding_tree ) {
    delete mp_expanding_tree;
    mp_expanding_tree = NULL;
  }
  mp_string_grammar = p_grammar;
  mp_expanding_tree = new ExpandingTree();
  mp_expanding_tree->init( p_grammar, p_worldmap );

  for( std::vector<ExpandingNode*>::iterator it = mp_expanding_tree->m_nodes.begin();
       it != mp_expanding_tree->m_nodes.end(); it ++ ) {
    ExpandingNode* p_node = (*it);
    
  } 

}
