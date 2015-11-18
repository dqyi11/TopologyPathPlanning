#ifndef EXPANDING_TREE_H
#define EXPANDING_TREE_H

#include <vector>
#include <string>
#include "kdtreeml2d.h"
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
    void import_ancestor_seq ( std::vector<ExpandingNode*> ancestor_seq );
    std::vector<std::string> get_substring();

    POS2D sample_random_pos();
    std::vector<POS2D> find_feasible_path( ExpandingEdge* p_in_edge, ExpandingEdge* p_out_edge );
 
    std::string                 m_name;
    ExpandingEdge*              mp_in_edge;
    std::vector<ExpandingEdge*> mp_out_edges;
    homotopy::SubRegion*        mp_subregion;

    std::vector<ExpandingNode*> mp_ancestor_seq;
  };
  
  class ExpandingEdge {
  public:
    ExpandingEdge( std::string name );
    virtual ~ExpandingEdge();
    
    void import_ancestor_seq ( std::vector<ExpandingEdge*> ancestor_seq );
    std::vector<std::string> get_substring();

    POS2D sample_random_pos();

    std::string                 m_name;
    ExpandingNode*              mp_from;
    ExpandingNode*              mp_to;
    homotopy::LineSubSegment*   mp_line_subsegment;

    POS2D                       m_rand_pos;
    std::vector<ExpandingEdge*> mp_ancestor_seq;
  };

  class ExpandingTree {
  public:
    ExpandingTree();
    virtual ~ExpandingTree();

    bool init( homotopy::StringGrammar* p_grammar, homotopy::WorldMap* p_worldmap = NULL );

    void output( std::string filename );

    int get_index( ExpandingNode* p_node ); 
    std::vector<ExpandingNode*> get_leaf_nodes();   
  
    ExpandingNode* mp_root; 
    std::vector<ExpandingNode*> m_nodes;
    std::vector<ExpandingEdge*> m_edges;
  };
}

#endif /* EXPANDING_TREE_H */
