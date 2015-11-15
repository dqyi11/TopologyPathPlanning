#include <iostream>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/iteration_macros.hpp>
#include "expanding_tree.h"


using namespace std;
using namespace boost;
using namespace mlrrts;

struct Vertex{ std::string name; };
struct Edge{ std::string name; };

typedef adjacency_list<vecS, vecS, undirectedS, Vertex, Edge> Graph;
typedef graph_traits<Graph>::vertex_descriptor vertex_t;
typedef graph_traits<Graph>::edge_descriptor edge_t;

ExpandingNode::ExpandingNode( string name ) {
  m_name = name;
  m_out_edges.clear();
}

ExpandingNode::~ExpandingNode() {

  m_out_edges.clear();
}

ExpandingEdge* ExpandingNode::find_out_edge( std::string name ) {

  ExpandingEdge* p_edge = NULL;
  for(vector<ExpandingEdge*>::iterator it = m_out_edges.begin(); it != m_out_edges.end(); it++ ) {
    ExpandingEdge* p_current_edge = (*it);
    if( p_current_edge->m_name == name ) {
      return p_current_edge;
    } 
  }
  return p_edge;
}

ExpandingEdge::ExpandingEdge( string name ) {
  m_name = name;
  mp_from = NULL;
  mp_to = NULL;
}

ExpandingEdge::~ExpandingEdge() {

}

bool ExpandingNode::has_out_edge( ExpandingEdge* p_edge ) {

  for(vector<ExpandingEdge*>::iterator it = m_out_edges.begin(); it != m_out_edges.end(); it++ ) {
    ExpandingEdge* p_current_edge = (*it);
    if( p_current_edge == p_edge ) {
      return true;
    } 
  }
  return false;
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
    ExpandingNode* p_current_node = NULL;
    for( unsigned int j = 0; j < path.size(); j ++ ) {
      homotopy::Adjacency adj = path[ j ];
      if ( p_current_node == NULL ) {
        /* first node */
        if( _p_root == NULL ) {
          /* root uninitialized */
          _p_root = new ExpandingNode();
          _p_root->m_name = adj.mp_state->m_name;
        }
        else {
          /* root initialized */
          p_current_node = _p_root;
          if( p_current_node->m_name != adj.mp_state->m_name ) {
            cout << "ERROR " << endl;
          }
        }                
      } 
      else {
        /* not first node */
        ExpandingEdge* p_edge = p_current_node->find_out_edge( adj.mp_transition->m_name );
        if( p_edge == NULL ) {
          /* no edge found */ 
          p_edge = new ExpandingEdge();
          p_edge->m_name = adj.mp_transition->m_name;
          p_edge->mp_from = p_current_node;
          p_edge->mp_to = new ExpandingNode();
          p_edge->mp_to->m_name = adj.mp_state->m_name;

          p_current_node = p_edge->mp_to;
        }
        else {
          /* edge found */
          if( p_edge->m_name == adj.mp_transition->m_name &&
              p_edge->mp_to->m_name == adj.mp_state->m_name ) {
            p_current_node = p_edge->mp_to;
          }     
          else {
            cout << "ERROR [EDGE MISMATCH] E(" << p_edge->m_name << " || " << adj.mp_transition->m_name << ")";
            cout << " N(" << p_edge->mp_to->m_name << " || " << adj.mp_transition->m_name << ")" << endl;
          }
        }   
      }
    }
  }  
  return true;
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
  Graph g;

  ofstream dot( filename.c_str() );
  
}  
