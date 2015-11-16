#ifndef EXPANDING_TREE_H
#define EXPANDING_TREE_H

#include <vector>
#include <string>
#include "string_grammar.h"
#include "worldmap.h"

namespace mlrrts {

  class ExpandingEdge;

  class ExpandingNode {
  public:
    ExpandingNode( std::string name );
    virtual ~ExpandingNode();

    ExpandingEdge* find_out_edge( std::string name );
    bool has_out_edge( ExpandingEdge* p_edge );

    ExpandingNode* get_parent_node();
    std::vector<ExpandingNode*> get_child_nodes();
 
    std::string                 m_name;
    ExpandingEdge*              mp_in_edge;
    std::vector<ExpandingEdge*> mp_out_edges;
    homotopy::SubRegion*        mp_subregion;
  };
  
  class ExpandingEdge {
  public:
    ExpandingEdge( std::string name );
    virtual ~ExpandingEdge();

    std::string                m_name;
    ExpandingNode*             mp_from;
    ExpandingNode*             mp_to;
    homotopy::LineSubSegment*  mp_linesubsegment;
  };

  class ExpandingTree {
  public:
    ExpandingTree();
    virtual ~ExpandingTree();

    bool init( homotopy::StringGrammar * p_grammar, homotopy::WorldMap* p_worldmap = NULL );

    void output( std::string filename );

    int get_index( ExpandingNode* p_node );    
  
    ExpandingNode* mp_root; 
    std::vector<ExpandingNode*> m_nodes;
    std::vector<ExpandingEdge*> m_edges;
  };
}

#endif /* EXPANDING_TREE_H */
