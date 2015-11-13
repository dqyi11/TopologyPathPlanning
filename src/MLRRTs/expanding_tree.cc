#include "expanding_tree.h"

using namespace mlrrts;

ExpandingNode::ExpandingNode() {

  m_child_nodes.clear();
}

ExpandingNode::~ExpandingNode() {

  m_child_nodes.clear();
}


ExpandingTree::ExpandingTree() {

  m_nodes.clear();
}

ExpandingTree::~ExpandingTree() {

  m_nodes.clear();
}

bool ExpandingTree::init( homotopy::StringGrammar * p_grammar ) {
  if( p_grammar == NULL ) {
    return false;
  }

  std::vector< std::vector < homotopy::Adjacency > > paths = p_grammar->find_simple_paths();

  return true;
}

