#ifndef EXPANDING_TREE_H
#define EXPANDING_TREE_H

#include <vector>
#include <string>
#include "string_grammar.h"

namespace mlrrts {

  class ExpandingNode {
  public:
    ExpandingNode();
    virtual ~ExpandingNode();

    std::string m_name;
    std::vector<ExpandingNode*> m_child_nodes;
  };

  class ExpandingTree {
  public:
    ExpandingTree();
    virtual ~ExpandingTree();

    bool init( homotopy::StringGrammar * p_grammar );
   
    std::vector<ExpandingNode*> m_nodes;
  };
}

#endif /* EXPANDING_TREE_H */
