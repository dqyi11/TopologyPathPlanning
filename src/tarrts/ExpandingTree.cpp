#include <iostream>
#include <limits>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/iteration_macros.hpp>
#include "topologyPathPlanning/tarrts/ExpandingTree.hpp"
#include "topologyPathPlanning/tarrts/ExpandingTreeMgr.hpp"
#include "topologyPathPlanning/tarrts/MLRRTstar.hpp"
#include "topologyPathPlanning/tarrts/MLUtil.hpp"

using namespace std;
using namespace boost;
using namespace topologyPathPlanning::homotopy;

namespace topologyPathPlanning {

namespace tarrts {

#define MAX_VAL numeric_limits<float>::max()

struct Vertex{ std::string name; };
struct Edge{ std::string name; };

typedef adjacency_list<vecS, vecS, undirectedS, Vertex, Edge> Graph;
typedef graph_traits<Graph>::vertex_descriptor vertex_t;
typedef graph_traits<Graph>::edge_descriptor edge_t;

StringClass::StringClass( std::vector< std::string > string ) {
  m_string = string;
  mp_kd_tree = new KDTree2D( std::ptr_fun(tac) );
  mp_exp_nodes.clear(); 
  mp_reference_frames.clear();  
  
  mp_path = NULL;
  m_cost = MAX_VAL;
  m_created_iteration_num = 0;
}

StringClass::~StringClass() {
  m_string.clear();
  if( mp_kd_tree ) {
    delete mp_kd_tree;
    mp_kd_tree = NULL;
  }
  mp_exp_nodes.clear(); 
  mp_reference_frames.clear();  
  
  m_cost = 0.0;
  mp_path = NULL;
}

void StringClass::init( homotopy::ReferenceFrameSet* p_rfs ) {
  if( p_rfs ) {
    for( unsigned int i=0; i < m_string.size(); i++ ) {
      string id = m_string[i];
      ReferenceFrame* p_rf = p_rfs->getReferenceFrame( id );
      if( p_rf ) {
        mp_reference_frames.push_back( p_rf );
      }
    }
  }
}

void StringClass::import( Path* p_path ) {
  if(p_path) {
    if( mp_path == NULL ) {
      mp_path = p_path;
      m_cost = p_path->mCost;
    }
    else {
      if(p_path->mCost < m_cost) {
        mp_path = p_path;
        m_cost = p_path->mCost;
      }
    }
  }
}

std::string StringClass::getName() {
  std::string name = "";
  for( unsigned int i = 0; i < m_string.size(); i ++ ) {
    if (i > 0) { 
      name += " ";
    }
    name += m_string[i];
  }
  return name;
}

void StringClass::addExpNode( ExpandingNode* p_node ) {
  if( p_node ) {
    mp_exp_nodes.push_back( p_node );
    p_node->mpStringClasses.push_back( this );
  }
}

void StringClass::dumpHistoricalData( std::string filename ) {
  std::ofstream hist_data_file;
  hist_data_file.open(filename.c_str());
  hist_data_file << getName() << std::endl;
  writeHistoricalData( hist_data_file );
  hist_data_file.close();
}

void StringClass::writeHistoricalData( std::ostream& out ) {

  for(std::vector<double>::iterator it = m_historical_data.begin();
      it != m_historical_data.end(); it++ ) {
    double data = (*it);
    out << data << " ";
  }
  out << std::endl;
  for(int i=m_created_iteration_num;
      i<m_historical_data.size()+m_created_iteration_num;i++) {
    out << i << " ";
  }
  out << std::endl;
}

void StringClass::record() {
  
  if(mp_path==NULL) {
    m_created_iteration_num ++;
  }
  else {
    m_historical_data.push_back(m_cost);
  }
}

ExpandingNode::ExpandingNode( string name ) {
  mName = name;
  mpInEdge = NULL;
  mpSubregion = NULL;
  mpStringClasses.clear();
  mpOutEdges.clear();
}

ExpandingNode::~ExpandingNode() {
  mpInEdge = NULL;
  mpSubregion = NULL;
  mpStringClasses.clear();
  mpOutEdges.clear();
}

ExpandingNode* ExpandingNode::getParentNode() {
  if( mpInEdge ) {
    return mpInEdge->mpFrom; 
  }
  return NULL;
}

std::vector<ExpandingNode*> ExpandingNode::getChildNodes() {
  std::vector<ExpandingNode*> nodes;
  for( unsigned int i = 0; i < mpOutEdges.size(); i ++ ) {
    ExpandingNode* p_node = mpOutEdges[i]->mpTo;
    if( p_node ) {
      nodes.push_back( p_node );
    }
  }
  return nodes;
}

POS2D ExpandingNode::sampleRandomPos() {
  POS2D pos( 0, 0 );
  if( mpSubregion ) {
    Point2D point = mpSubregion->samplePosition();
    pos = toPOS2D( point );
  }
  return pos;
}

std::vector<POS2D> ExpandingNode::findFeasiblePath( ExpandingEdge* p_in_edge, ExpandingEdge* p_out_edge ) {
  std::vector<POS2D> paths;
  if( p_in_edge && p_out_edge ) {

  }  
  return paths;
}

ExpandingEdge* ExpandingNode::findOutEdge( std::string name ) {

  ExpandingEdge* p_edge = NULL;
  for(vector<ExpandingEdge*>::iterator it = mpOutEdges.begin(); it != mpOutEdges.end(); it++ ) {
    ExpandingEdge* p_current_edge = (*it);
    if( p_current_edge->mName == name ) {
      return p_current_edge;
    } 
  }
  return p_edge;
}
   
void ExpandingNode::importAncestorSeq ( std::vector<ExpandingNode*> ancestor_seq ) {
  mpAncestorSeq.clear();
  for( unsigned int i = 0; i < ancestor_seq.size(); i++ ) {
    ExpandingNode* p_node = ancestor_seq[i ];
    mpAncestorSeq.push_back( p_node );
  }
}

bool ExpandingNode::hasOutEdge( ExpandingEdge* p_edge ) {

  for(vector<ExpandingEdge*>::iterator it = mpOutEdges.begin(); it != mpOutEdges.end(); it++ ) {
    ExpandingEdge* p_current_edge = (*it);
    if( p_current_edge == p_edge ) {
      return true;
    } 
  }
  return false;
}

std::vector<std::string> ExpandingNode::getSubstring() {
  std::vector<std::string> substring;
  for( unsigned int i = 0; i < mpAncestorSeq.size(); i++ ) {
    substring.push_back( mpAncestorSeq[i]->mName );
  }
  return substring;
}

bool ExpandingNode::isAncestor( ExpandingNode* p_node ) {
  for( vector<ExpandingNode*>::iterator it = mpAncestorSeq.begin();
       it != mpAncestorSeq.end(); it++) {
    ExpandingNode* p_ancestor_node = (*it);
    if( p_ancestor_node == p_node ) {
      return false;
    }
  }
  return false;
}

ExpandingEdge::ExpandingEdge( string name ) {
  mName = name;
  mpFrom = NULL;
  mpTo = NULL;
  mpLineSubsegment = NULL;
  mpReferenceFrame = NULL;
}

ExpandingEdge::~ExpandingEdge() {
  mpFrom = NULL;
  mpTo = NULL;
  mpLineSubsegment = NULL;
  mpReferenceFrame = NULL;
}

void ExpandingEdge::importAncestorSeq ( std::vector<ExpandingEdge*> ancestor_seq ) {
  mpAncestorSeq.clear();
  for( unsigned int i = 0; i < ancestor_seq.size(); i++ ) {
    ExpandingEdge* p_edge = ancestor_seq[i ];
    mpAncestorSeq.push_back( p_edge );
  }
}

std::vector<std::string> ExpandingEdge::getSubstring() {
  std::vector<std::string> substring;
  for( unsigned int i = 0; i < mpAncestorSeq.size(); i++ ) {
    substring.push_back( mpAncestorSeq[i]->mName );
  }
  return substring;
}

POS2D ExpandingEdge::sampleRandomPos() {
  POS2D pos( 0, 0 );
  if( mpLineSubsegment ) {
    Point2D point = mpLineSubsegment->samplePosition();
    pos = toPOS2D( point );
  }
  return pos;
}

ExpandingTree::ExpandingTree() {

  mpRoot = NULL;
  mNodes.clear();
}

ExpandingTree::~ExpandingTree() {
  
  mpRoot = NULL;
  mNodes.clear();
}

std::vector< StringClass* > ExpandingTree::init( homotopy::StringGrammar * p_grammar, homotopy::ReferenceFrameSet* p_reference_frame_set ) {
  std::vector< StringClass* > string_classes;
  if( p_grammar == NULL || p_reference_frame_set == NULL ) {
    return string_classes;
  }

  std::vector< std::vector < homotopy::Adjacency > > paths;
  if( p_reference_frame_set->getStringConstraint().size() > 0 ) {  
    paths = p_grammar->findPaths( p_reference_frame_set->getStringConstraint() );
  }
  else {
    paths = p_grammar->findSimplePaths();
  }

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
    p_string_class->init( p_reference_frame_set );
   
    for( unsigned int j = 0; j < path.size(); j ++ ) {
      homotopy::Adjacency adj = path[ j ];
      if ( p_current_node == NULL ) {
        /* first node */
        if( mpRoot == NULL ) {
          /* root uninitialized */
          mpRoot = new ExpandingNode( adj.mp_state->mName );
          mpRoot->importAncestorSeq( node_seq );
          node_seq.push_back( mpRoot );
          if ( p_reference_frame_set 
               && p_reference_frame_set->getWorldMap() ) {
            mpRoot->mpSubregion = p_reference_frame_set->getWorldMap()->findSubregion( adj.mp_state->mName );
          }
          mNodes.push_back( mpRoot );
          p_string_class->addExpNode( mpRoot );
          p_current_node = mpRoot;
        }
        else {
          /* root initialized */
          p_string_class->addExpNode( mpRoot );
          p_current_node = mpRoot;
          if( p_current_node->mName != adj.mp_state->mName ) {
            cout << "ERROR [ROOT MISMATCH] Root Name=\"" << p_current_node->mName <<"\" Adj State Name =\"" << adj.mp_state->mName << "\""  << endl;
          }
          node_seq.push_back( mpRoot );
        }                
      } 
      else {
        /* not first node */
        ExpandingEdge* p_edge = p_current_node->findOutEdge( adj.mp_transition->m_name );
        if( p_edge == NULL ) {
          /* no edge found */ 
          p_edge = new ExpandingEdge( adj.mp_transition->m_name );
          p_edge->mpFrom = p_current_node;
          p_edge->importAncestorSeq( edge_seq );
          edge_seq.push_back( p_edge );
          if ( p_reference_frame_set 
               && p_reference_frame_set->getWorldMap() ) {
            p_edge->mpLineSubsegment = p_reference_frame_set->getWorldMap()->findLinesubsegment( adj.mp_transition->m_name );
            p_edge->mpReferenceFrame = p_reference_frame_set->getReferenceFrame( adj.mp_transition->m_name );
          }
          p_edge->mpTo = new ExpandingNode( adj.mp_state->mName );
          p_edge->mpTo->mpInEdge = p_edge;
          p_edge->mpFrom->mpOutEdges.push_back( p_edge );
          p_edge->mpTo->importAncestorSeq( node_seq );
          node_seq.push_back( p_edge->mpTo );
          p_string_class->addExpNode( p_edge->mpTo );
          if ( p_reference_frame_set 
               && p_reference_frame_set->getWorldMap() ) {
            p_edge->mpTo->mpSubregion = p_reference_frame_set->getWorldMap()->findSubregion( adj.mp_state->mName );
          }
          mEdges.push_back( p_edge );
          mNodes.push_back( p_edge->mpTo );
 
          p_current_node = p_edge->mpTo;
        }
        else {
          /* edge found */
          if( p_edge->mName == adj.mp_transition->m_name &&
              p_edge->mpTo->mName == adj.mp_state->mName ) {
            edge_seq.push_back( p_edge );
            node_seq.push_back( p_edge->mpTo );
            p_string_class->addExpNode( p_edge->mpTo );
            p_current_node = p_edge->mpTo;
          }     
          else {
            cout << "ERROR [EDGE MISMATCH] E(" << p_edge->mName << " || " << adj.mp_transition->m_name << ")";
            cout << " N(" << p_edge->mpTo->mName << " || " << adj.mp_transition->m_name << ")" << endl;
          }
        }   
      }
    }
    string_classes.push_back( p_string_class );
  } 
 
  return string_classes;
}

int ExpandingTree::getIndex( ExpandingNode* p_node ) {

  for( int i = 0; i < mNodes.size(); i++ ) {
    if( mNodes[i] == p_node ) {
      return i;
    }
  } 
  return -1;
}

void ExpandingTree::output( std::string filename ) {

  const unsigned int edge_num = mEdges.size();
  const unsigned int vertex_num = mNodes.size();
 
  Graph g;
  std::vector<vertex_t> vs;
  for( unsigned int i = 0; i < vertex_num; i ++ ) {
    vertex_t vt = add_vertex( g );
    g[vt].name = mNodes[ i ]->mName;
    vs.push_back( vt );
  }
  std::vector<edge_t> es;
  for( unsigned int i = 0; i < edge_num; i ++ ) {
    bool b = false;
    edge_t et;
    ExpandingEdge* p_edge = mEdges[ i ];
    int s_i = getIndex( p_edge->mpFrom );
    int g_i = getIndex( p_edge->mpTo );
    tie(et, b) = add_edge( vs[s_i], vs[g_i], g );
    g[et].name = p_edge->mName;
    es.push_back( et );
  }
 
  ofstream dot( filename.c_str() );
  write_graphviz( dot, g, make_label_writer( get( &Vertex::name, g) ), make_label_writer( get( &Edge::name, g ) ) );
 
}  

std::vector<ExpandingNode*> ExpandingTree::getLeafNodes() {
  std::vector<ExpandingNode*> leaf_nodes;
  for( std::vector<ExpandingNode*>::iterator it = mNodes.begin();
       it != mNodes.end(); it++ ) {
    ExpandingNode* p_node = (*it);
    if (p_node->mpOutEdges.size() == 0) {
      leaf_nodes.push_back( p_node );
    }
  }
  return leaf_nodes;
} 

void ExpandingTree::print() {
  std::cout << "NODE " << std::endl;
  for( unsigned int i=0; i < mNodes.size(); i++ ) {
    ExpandingNode* p_node = mNodes[i];
    if(p_node) {
      std::cout << p_node->mName << " " << p_node->mpSubregion->getName() << " " << std::endl;
    }
  }
  std::cout << "EDGE " << std::endl;
  for( unsigned int i=0; i < mEdges.size(); i++ ) {
    ExpandingEdge* p_edge = mEdges[i];
    if(p_edge) {
      std::cout << p_edge->mName << " " << p_edge->mpReferenceFrame->getName() << " " << p_edge->mpLineSubsegment->getName() << std::endl;
    }
  }
}

} // tarrts

} // topologyPathPlanning
