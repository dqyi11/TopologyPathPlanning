#ifndef EXPANDING_TREE_H
#define EXPANDING_TREE_H

#include <vector>
#include <string>
#include "string_grammar.h"

namespace mlrrts {

  class ExpandingEdge;

  class ExpandingNode {
  public:
    ExpandingNode( std::string name );
    virtual ~ExpandingNode();

    ExpandingEdge* find_out_edge( std::string name );
    bool has_out_edge( ExpandingEdge* p_edge );
 
    std::string m_name;
    std::vector<ExpandingEdge*> m_out_edges;
  };
  
  class ExpandingEdge {
  public:
    ExpandingEdge( std::string name );
    virtual ~ExpandingEdge();

    std::string m_name;
    ExpandingNode* mp_from;
    ExpandingNode* mp_to;
  };

  class ExpandingTree {
  public:
    ExpandingTree();
    virtual ~ExpandingTree();

    bool init( homotopy::StringGrammar * p_grammar );

    void output( std::string filename );

    int get_index( ExpandingNode* p_node );    
  
    ExpandingNode* _p_root; 
    std::vector<ExpandingNode*> m_nodes;
  };
}

#endif /* EXPANDING_TREE_H */
