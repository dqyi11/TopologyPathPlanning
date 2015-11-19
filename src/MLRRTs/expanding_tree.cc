#include <iostream>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/iteration_macros.hpp>
#include "expanding_tree.h"
#include "expanding_tree_mgr.h"
#include "ml_util.h"

using namespace std;
using namespace boost;
using namespace mlrrts;

struct Vertex{ std::string name; };
struct Edge{ std::string name; };

typedef adjacency_list<vecS, vecS, undirectedS, Vertex, Edge> Graph;
typedef graph_traits<Graph>::vertex_descriptor vertex_t;
typedef graph_traits<Graph>::edge_descriptor edge_t;

StringClass::StringClass( std::vector< std::string > string ) {
  m_string = string;
  mp_kd_tree = new KDTree2D( std::ptr_fun(tac) );
  mp_exp_nodes.clear(); 
 
  mp_path = NULL;
  m_cost = 0.0;
}

StringClass::~StringClass() {
  m_string.clear();
  if( mp_kd_tree ) {
    delete mp_kd_tree;
    mp_kd_tree = NULL;
  }
  mp_exp_nodes.clear(); 
  
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

void StringClass::add_exp_node( ExpandingNode* p_node ) {
  if( p_node ) {
    mp_exp_nodes.push_back( p_node );
    p_node->mp_string_classes.push_back( this );
  }
}

ExpandingNode::ExpandingNode( string name ) {
  m_name = name;
  mp_in_edge = NULL;
  mp_subregion = NULL;
  mp_string_classes.clear();
  mp_out_edges.clear();
}

ExpandingNode::~ExpandingNode() {
  mp_in_edge = NULL;
  mp_subregion = NULL;
  mp_string_classes.clear();
  mp_out_edges.clear();
}

ExpandingNode* ExpandingNode::get_parent_node() {
  if( mp_in_edge ) {
    return mp_in_edge->mp_from; 
  }
  return NULL;
}

std::vector<ExpandingNode*> ExpandingNode::get_child_nodes() {
  std::vector<ExpandingNode*> nodes;
  for( unsigned int i = 0; i < mp_out_edges.size(); i ++ ) {
    ExpandingNode* p_node = mp_out_edges[i]->mp_to;
    if( p_node ) {
      nodes.push_back( p_node );
    }
  }
  return nodes;
}

POS2D ExpandingNode::sample_random_pos() {
  POS2D pos( 0, 0 );
  if( mp_subregion ) {
    Point2D point = mp_subregion->sample_position();
    pos = toPOS2D( point );
  }
  return pos;
}

std::vector<POS2D> ExpandingNode::find_feasible_path( ExpandingEdge* p_in_edge, ExpandingEdge* p_out_edge ) {
  std::vector<POS2D> paths;
  if( p_in_edge && p_out_edge ) {

  }  
  return paths;
}

ExpandingEdge* ExpandingNode::find_out_edge( std::string name ) {

  ExpandingEdge* p_edge = NULL;
  for(vector<ExpandingEdge*>::iterator it = mp_out_edges.begin(); it != mp_out_edges.end(); it++ ) {
    ExpandingEdge* p_current_edge = (*it);
    if( p_current_edge->m_name == name ) {
      return p_current_edge;
    } 
  }
  return p_edge;
}
   
void ExpandingNode::import_ancestor_seq ( std::vector<ExpandingNode*> ancestor_seq ) {
  mp_ancestor_seq.clear();
  for( unsigned int i = 0; i < ancestor_seq.size(); i++ ) {
    ExpandingNode* p_node = ancestor_seq[i ];
    mp_ancestor_seq.push_back( p_node );
  }
}

bool ExpandingNode::has_out_edge( ExpandingEdge* p_edge ) {

  for(vector<ExpandingEdge*>::iterator it = mp_out_edges.begin(); it != mp_out_edges.end(); it++ ) {
    ExpandingEdge* p_current_edge = (*it);
    if( p_current_edge == p_edge ) {
      return true;
    } 
  }
  return false;
}

std::vector<std::string> ExpandingNode::get_substring() {
  std::vector<std::string> substring;
  for( unsigned int i = 0; i < mp_ancestor_seq.size(); i++ ) {
    substring.push_back( mp_ancestor_seq[i]->m_name );
  }
  return substring;
}

bool ExpandingNode::is_ancestor( ExpandingNode* p_node ) {
  for( vector<ExpandingNode*>::iterator it = mp_ancestor_seq.begin();
       it != mp_ancestor_seq.end(); it++) {
    ExpandingNode* p_ancestor_node = (*it);
    if( p_ancestor_node == p_node ) {
      return false;
    }
  }
  return false;
}

ExpandingEdge::ExpandingEdge( string name ) {
  m_name = name;
  mp_from = NULL;
  mp_to = NULL;
  mp_line_subsegment = NULL;
}

ExpandingEdge::~ExpandingEdge() {
  mp_from = NULL;
  mp_to = NULL;
  mp_line_subsegment = NULL;
}

void ExpandingEdge::import_ancestor_seq ( std::vector<ExpandingEdge*> ancestor_seq ) {
  mp_ancestor_seq.clear();
  for( unsigned int i = 0; i < ancestor_seq.size(); i++ ) {
    ExpandingEdge* p_edge = ancestor_seq[i ];
    mp_ancestor_seq.push_back( p_edge );
  }
}

std::vector<std::string> ExpandingEdge::get_substring() {
  std::vector<std::string> substring;
  for( unsigned int i = 0; i < mp_ancestor_seq.size(); i++ ) {
    substring.push_back( mp_ancestor_seq[i]->m_name );
  }
  return substring;
}

POS2D ExpandingEdge::sample_random_pos() {
  POS2D pos( 0, 0 );
  if( mp_line_subsegment ) {
    Point2D point = mp_line_subsegment->sample_position();
    pos = toPOS2D( point );
  }
  return pos;
}

ExpandingTree::ExpandingTree() {

  mp_root = NULL;
  m_nodes.clear();
}

ExpandingTree::~ExpandingTree() {
  
  mp_root = NULL;
  m_nodes.clear();
}

std::vector< StringClass* > ExpandingTree::init( homotopy::StringGrammar * p_grammar, homotopy::WorldMap* p_worldmap ) {
  std::vector< StringClass* > string_classes;
  if( p_grammar == NULL ) {
    return string_classes;
  }

  std::vector< std::vector < homotopy::Adjacency > > paths = p_grammar->find_simple_paths();

  for( unsigned int i = 0; i < paths.size(); i ++ ) {
    std::vector< homotopy::Adjacency > path = paths[ i ];
    
    std::vector< std::string > string_class;
    ExpandingNode* p_current_node = NULL;
    std::vector<ExpandingNode*> node_seq;
    std::vector<ExpandingEdge*> edge_seq;

    for( unsigned int j = 0; j < path.size(); j ++ ) {
      homotopy::Adjacency adj = path[ j ];
      if( adj.mp_transition ) {
        string_class.push_back( adj.mp_transition->m_name );
      }
    }
    StringClass* p_string_class = new StringClass( string_class );
   
    for( unsigned int j = 0; j < path.size(); j ++ ) {
      homotopy::Adjacency adj = path[ j ];
      if ( p_current_node == NULL ) {
        /* first node */
        if( mp_root == NULL ) {
          /* root uninitialized */
          mp_root = new ExpandingNode( adj.mp_state->m_name );
          mp_root->import_ancestor_seq( node_seq );
          node_seq.push_back( mp_root );
          if ( p_worldmap ) {
            mp_root->mp_subregion = p_worldmap->find_subregion( adj.mp_state->m_name );
          }
          m_nodes.push_back( mp_root );
          p_string_class->add_exp_node( mp_root );
          p_current_node = mp_root;
        }
        else {
          /* root initialized */
          p_string_class->add_exp_node( mp_root );
          p_current_node = mp_root;
          if( p_current_node->m_name != adj.mp_state->m_name ) {
            cout << "ERROR [ROOT MISMATCH] Root Name=\"" << p_current_node->m_name <<"\" Adj State Name =\"" << adj.mp_state->m_name << "\""  << endl;
          }
          node_seq.push_back( mp_root );
        }                
      } 
      else {
        /* not first node */
        ExpandingEdge* p_edge = p_current_node->find_out_edge( adj.mp_transition->m_name );
        if( p_edge == NULL ) {
          /* no edge found */ 
          p_edge = new ExpandingEdge( adj.mp_transition->m_name );
          p_edge->mp_from = p_current_node;
          p_edge->import_ancestor_seq( edge_seq );
          edge_seq.push_back( p_edge );
          if( p_worldmap ) {
            p_edge->mp_line_subsegment = p_worldmap->find_linesubsegment( adj.mp_transition->m_name );
          }
          p_edge->mp_to = new ExpandingNode( adj.mp_state->m_name );
          p_edge->mp_to->mp_in_edge = p_edge;
          p_edge->mp_from->mp_out_edges.push_back( p_edge );
          p_edge->mp_to->import_ancestor_seq( node_seq );
          node_seq.push_back( p_edge->mp_to );
          p_string_class->add_exp_node( p_edge->mp_to );
          if( p_worldmap ) {
            p_edge->mp_to->mp_subregion = p_worldmap->find_subregion( adj.mp_state->m_name );
          }
          m_edges.push_back( p_edge );
          m_nodes.push_back( p_edge->mp_to );
 
          p_current_node = p_edge->mp_to;
        }
        else {
          /* edge found */
          if( p_edge->m_name == adj.mp_transition->m_name &&
              p_edge->mp_to->m_name == adj.mp_state->m_name ) {
            edge_seq.push_back( p_edge );
            node_seq.push_back( p_edge->mp_to );
            p_string_class->add_exp_node( p_edge->mp_to );
            p_current_node = p_edge->mp_to;
          }     
          else {
            cout << "ERROR [EDGE MISMATCH] E(" << p_edge->m_name << " || " << adj.mp_transition->m_name << ")";
            cout << " N(" << p_edge->mp_to->m_name << " || " << adj.mp_transition->m_name << ")" << endl;
          }
        }   
      }
    }
    string_classes.push_back( p_string_class );
  } 
 
  return string_classes;
}

int ExpandingTree::get_index( ExpandingNode* p_node ) {

  for( int i = 0; i < m_nodes.size(); i++ ) {
    if( m_nodes[i] == p_node ) {
      return i;
    }
  } 
  return -1;
}

void ExpandingTree::output( std::string filename ) {

  const unsigned int edge_num = m_edges.size();
  const unsigned int vertex_num = m_nodes.size();
 
  Graph g;
  std::vector<vertex_t> vs;
  for( unsigned int i = 0; i < vertex_num; i ++ ) {
    vertex_t vt = add_vertex( g );
    g[vt].name = m_nodes[ i ]->m_name;
    vs.push_back( vt );
  }
  std::vector<edge_t> es;
  for( unsigned int i = 0; i < edge_num; i ++ ) {
    bool b = false;
    edge_t et;
    ExpandingEdge* p_edge = m_edges[ i ];
    int s_i = get_index( p_edge->mp_from );
    int g_i = get_index( p_edge->mp_to );
    tie(et, b) = add_edge( vs[s_i], vs[g_i], g );
    g[et].name = p_edge->m_name;
    es.push_back( et );
  }
 
  ofstream dot( filename.c_str() );
  write_graphviz( dot, g, make_label_writer( get( &Vertex::name, g) ), make_label_writer( get( &Edge::name, g ) ) );
 
}  

std::vector<ExpandingNode*> ExpandingTree::get_leaf_nodes() {
  std::vector<ExpandingNode*> leaf_nodes;
  for( std::vector<ExpandingNode*>::iterator it = m_nodes.begin();
       it != m_nodes.end(); it++ ) {
    ExpandingNode* p_node = (*it);
    if (p_node->mp_out_edges.size() == 0) {
      leaf_nodes.push_back( p_node );
    }
  }
  return leaf_nodes;
} 
