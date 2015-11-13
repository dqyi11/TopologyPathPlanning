#include "expanding_tree.h"

using namespace mlrrts;

ExpandingNode::ExpandingNode() {

  m_out_edges.clear();
}

ExpandingNode::~ExpandingNode() {

  m_out_edges.clear();
}


ExpandingTree::ExpandingTree() {

  _p_root = NULL;
  m_nodes.clear();
}

ExpandingTree::~ExpandingTree() {
  
  _p_root = NULL;
  m_nodes.clear();
}

bool ExpandingTree::init( homotopy::StringGrammar * p_grammar ) {
  if( p_grammar == NULL ) {
    return false;
  }

  std::vector< std::vector < homotopy::Adjacency > > paths = p_grammar->find_simple_paths();

  for( unsigned int i = 0; i < paths.size(); i ++ ) {
    std::vector< homotopy::Adjacency > path = paths[ i ];
    for( unsigned int j = 0; j < path.size(); j ++ ) {
            
    }
  }  
  return true;
}
