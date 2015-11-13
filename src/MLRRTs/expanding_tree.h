#ifndef EXPANDING_TREE_H
#define EXPANDING_TREE_H

#include <vector>
#include <string>
#include "string_grammar.h"

namespace mlrrts {

  class ExpandingEdge;

  class ExpandingNode {
  public:
    ExpandingNode();
    virtual ~ExpandingNode();

    std::string m_name;
    std::vector<ExpandingEdge*> m_out_edges;
  };
  
  class ExpandingEdge {
  public:

    ExpandingNode* mp_from;
    ExpandingNode* mp_to;
  };

  class ExpandingTree {
  public:
    ExpandingTree();
    virtual ~ExpandingTree();

    bool init( homotopy::StringGrammar * p_grammar );
  
    ExpandingNode* _p_root; 
    std::vector<ExpandingNode*> m_nodes;
  };
}

#endif /* EXPANDING_TREE_H */
