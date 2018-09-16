#include <fstream>
#include <list>
#include <vector>
#include <queue>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/iteration_macros.hpp>
#include "topologyPathPlanning/homotopy/StringGrammar.hpp"

using namespace std;
using namespace boost;

namespace topologyPathPlanning {

namespace homotopy {

struct Vertex{ string name; };
struct Edge{ string name; double weight; };

typedef adjacency_list<vecS, vecS, undirectedS, Vertex, Edge> Graph;
typedef graph_traits<Graph>::vertex_descriptor vertex_t;
typedef graph_traits<Graph>::edge_descriptor   edge_t;


State::State( string name ) {
  mName = name;
}

State::~State() {

}

bool State::operator==(const State& other) const {
  if ( mName == other.mName ) {
    return true;
  }
  return false;
}

/*
Transition* State::find_transition( string trans_name ) {
  for( vector<Transition*>::iterator it = m_transitions.begin();
       it != m_transitions.end(); it++ ) {
    Transition* p_ref_transition = (*it);
    if ( p_ref_transition->m_name == trans_name ) {
      return p_ref_transition;
    }
  }
  return NULL;
}*/

Adjacency State::findAdjacency( string trans_name ) {
  Adjacency find_adj;
  find_adj.mp_state = NULL;
  find_adj.mp_transition = NULL;

  for( vector<Adjacency>::iterator it = mAdjacencies.begin();
       it != mAdjacencies.end(); it++ ) {
    Adjacency ref_adjacency = (*it);
    if ( ref_adjacency.mp_transition == NULL ) {
      continue;
    }
    if ( ref_adjacency.mp_transition->m_name == trans_name ) {
      return ref_adjacency;
    }
  }
  return find_adj;  
}

Transition::Transition( State* p_from , State* p_to , string name ) {
  m_name = name;
  mp_from_state = p_from;
  mp_to_state = p_to;
}

Transition::~Transition() {

}

bool Transition::operator==(const Transition& other) const {
  if ( m_name == other.m_name ) {
    return true;
  }
  return false;
}

StringGrammar::StringGrammar() {

}

StringGrammar::~StringGrammar() {

}

State* StringGrammar::findState( string name ) {
  for( vector<State*>::iterator it = mStates.begin();
       it != mStates.end(); it++ ) {
    State* p_ref_state = (*it);
    if ( p_ref_state->mName == name ) {
      return p_ref_state;
    }
  }
  return NULL;
}
    
int StringGrammar::getStateIndex( string name ) {
  for( unsigned int i = 0; i < mStates.size(); i ++ ) {
    if( mStates[i]->mName == name ) {
      return i;
    }
  }
  return -1;
}

Transition* StringGrammar::findTransition( string name ) {
  for( vector<Transition*>::iterator it = mTransitions.begin();
       it != mTransitions.end(); it++ ) {
    Transition* p_ref_transition = (*it);
    if (p_ref_transition->m_name == name) {
      return p_ref_transition;
    }
  }
  return NULL;
}


bool StringGrammar::hasTransition( Transition* p_transition ) {
  for( vector<Transition*>::iterator it = mTransitions.begin();
       it != mTransitions.end(); it++ ) {
    Transition* p_ref_transition = (*it);
    if ((*p_ref_transition) == (*p_transition)) {
      return true;
    }
  }
  return false;
}

bool StringGrammar::hasState( State* p_state ) {
  for( vector<State*>::iterator it = mStates.begin();
       it != mStates.end(); it++ ) {
    State* p_ref_state = (*it);
    if ( (*p_ref_state)==(*p_state) ) {
      return true;
    }
  }
  return false;
}

bool StringGrammar::addTransition( string from_name, string to_name, string name ) {
  Transition* p_transition = findTransition( name );
  if ( p_transition ) {
    return false;
  }
  State* p_from_state = findState( from_name );
  if ( p_from_state == NULL ) {
    p_from_state = new State( from_name );
    mStates.push_back(p_from_state);
  }
  State* p_to_state = findState( to_name );
  if ( p_to_state == NULL ) {
    p_to_state = new State( to_name );
    mStates.push_back(p_to_state);
  }

  p_transition = new Transition( p_from_state, p_to_state, name );
  mTransitions.push_back( p_transition );
  //p_from_state->m_transitions.push_back( p_transition );
  Adjacency from_adjacency;
  from_adjacency.mp_transition = p_transition;
  from_adjacency.mp_state = p_to_state; 
  p_from_state->mAdjacencies.push_back( from_adjacency );
  //p_to_state->m_transitions.push_back( p_transition );
  Adjacency to_adjacency;
  to_adjacency.mp_transition = p_transition;
  to_adjacency.mp_state = p_from_state; 
  p_to_state->mAdjacencies.push_back( to_adjacency );
  return true;
}

bool StringGrammar::setInit( string name ) {
  State* p_state = findState( name );
  if ( p_state == NULL ) {
    return false;
  }
  mpInitState = p_state;
  return true;
}

bool StringGrammar::setGoal( string name ) {
  State* p_state = findState( name );
  if ( p_state == NULL ) {
    return false;
  }
  mpGoalState = p_state;
  return true;
}

bool StringGrammar::isValidSubstring( vector< string > substr ) {
  State* p_current_state = mpInitState;
  for ( vector< string >::iterator it = substr.begin();
        it != substr.end(); it++ ) {
    string name = (*it);
    Adjacency adj = p_current_state->findAdjacency( name );
    if ( adj.mp_transition == NULL ) {
      return false;
    }
    p_current_state = adj.mp_state;
  }
  return true;
}

bool StringGrammar::isValidString( vector< string > str ) {
  unsigned int str_num = str.size();
  State* p_current_state = mpInitState;
  for ( unsigned int i=0; i < str_num; i++ ) {
    string name = str[i];
    Adjacency adj = p_current_state->findAdjacency( name );
    if ( adj.mp_transition == NULL ) {
      return false;
    }
    if( i < str_num-1 ) {
      p_current_state = adj.mp_state;
    }
    else {
      if ( adj.mp_state == mpGoalState ) {
        return true;
      }
    }
  }
  return false;
}

bool StringGrammar::isEquivalent( vector< string > str_a , vector< string > str_b ) {
  vector< string > str_a_stack;
  vector< string > str_b_stack;
  
  str_a_stack = getNonRepeatingForm( str_a );
  str_b_stack = getNonRepeatingForm( str_b );
  
  if( str_a_stack.size() != str_b_stack.size() ) {
    return false;
  }
  for( unsigned int i = 0; i < str_a_stack.size(); i ++ ) {
    if( str_a_stack[i] != str_b_stack[i] ) {
      return false;
    }
  }
  return true;
}

vector< string > StringGrammar::getNonRepeatingForm( vector< string > id_str ) {
  vector< string > str_non_repeat;
  for( unsigned int i = 0; i < id_str.size(); i ++ ) {
    string id = id_str[i];
    if ( str_non_repeat.size() > 0 && str_non_repeat.back() == id ) {
      str_non_repeat.pop_back();
    }
    else {
      str_non_repeat.push_back( id );
    }
  }
  return str_non_repeat;
}

void StringGrammar::output( string filename ) {
    
  const unsigned int edge_num = mTransitions.size();
  const unsigned int vertex_num = mStates.size();

  //cout << "STRING_GRAMMR::EDGE_NUM::" << edge_num << endl;
  //cout << "STRING_GRAMMR::VERTEX_NUM::" << vertex_num << endl;
  
  Graph g; 
  vector<vertex_t> vs;
  for( unsigned int i=0; i < vertex_num; i++) {
    vertex_t vt =  add_vertex( g );
    g[vt].name = mStates[i]->mName;
    vs.push_back(vt);
  }
  //cout << "Finish VERTEX INIT " << vs.size() << endl;
  vector<edge_t> es;
  for( unsigned int i=0; i < edge_num; i ++) {
    bool b = false;
    edge_t et;
    Transition* p_trans = mTransitions[i];
    int s_i = getStateIndex( p_trans->mp_from_state->mName );
    int g_i = getStateIndex( p_trans->mp_to_state->mName );
    cout << p_trans->m_name <<  " From " << s_i << " to "  << g_i << endl;
    tie(et, b) = add_edge( vs[s_i], vs[g_i], g );
    g[et].weight = 1.0;
    g[et].name = p_trans->m_name;
    es.push_back(et);
  }
  //cout << "Finish EDGE INIT " << es.size() << endl;
  ofstream dot( filename.c_str() );
  write_graphviz( dot, g , make_label_writer( get( &Vertex::name, g) ), make_label_writer( get( &Edge::name, g) ) );
  //cout << "WRITING " << filename << endl;
}

vector< vector< string > > StringGrammar::getAllSimpleStrings( ) {
  vector< vector< string > > simple_strings;
  if( mpInitState && mpGoalState ) { 
    simple_strings = findSimpleStrings();
  }
  return simple_strings;
}


bool is_adjacency_node_not_in_current_path(Adjacency node, vector< Adjacency > path ) {
  for(unsigned int i=0;i<path.size();i++) {
    /*
    if( path[i].mp_transition == NULL) {
      continue;
    }
    if(path[i].mp_transition->m_name == node.mp_transition->m_name) { 
      return false;
    }*/
    if(path[i].mp_state->mName == node.mp_state->mName) { 
      return false;
    }
  }
  return true;
}

void print_path( vector< Adjacency > path ) {
  cout << "PATH:";
  for(unsigned int i=0;i<path.size();i++) {
    cout << path[i].mp_state->mName.c_str() << " ";
  }
  cout << endl;
}

vector< vector< string > > StringGrammar::findSimpleStrings() {

  vector< vector< Adjacency > > path_list = findSimplePaths();
  vector< vector< string > > ids_list;
  for( unsigned int i=0;i<path_list.size(); i++) {
    vector< string > ids;
    vector< Adjacency > p = path_list[i];
    for( unsigned int j=0;j<p.size();j++) {
      Transition* p_transition = p[j].mp_transition;
      if( p_transition ) {
        ids.push_back( p_transition->m_name );
      }
    }
    ids_list.push_back( ids );
  }
  return ids_list;
}

vector< vector< Adjacency > > StringGrammar::findSimplePaths() {

  vector< vector< Adjacency > > path_list;
  vector< Adjacency > path;
  Adjacency init_adj;
  init_adj.mp_state = mpInitState;
  init_adj.mp_transition = NULL;  
  path.push_back(init_adj);

  queue< vector< Adjacency > > q;
  q.push(path);

  while(!q.empty()) {
    path = q.front();
    q.pop();

    // take the last node of a path
    Adjacency last_node_of_path = path[path.size()-1];
    
    // see if the last node of a path is the goal state
    if( last_node_of_path.mp_state->mName == mpGoalState->mName ) {
      print_path(path);
      path_list.push_back(path);
    }

    
    State* p_state = last_node_of_path.mp_state;
    if( p_state ) {
      // check the adjacency
      for(unsigned int i=0; i<p_state->mAdjacencies.size(); i++) {
        Adjacency adj = p_state->mAdjacencies[i];
        if(is_adjacency_node_not_in_current_path(adj, path)) {
          vector<Adjacency> new_path( path.begin(), path.end() );
          new_path.push_back( adj );
          q.push(new_path);
        }         
      }
    }
  }

  return path_list;
}

vector< vector< Adjacency > > StringGrammar::findPaths( vector< vector< string > > strs ) {

  vector< vector< Adjacency > > path_list;
  for( unsigned int i = 0; i < strs.size(); i++ ) {
    vector< string > str_id = strs[i];
    vector< Adjacency > path;
    Adjacency init_adj;
    init_adj.mp_state = mpInitState;
    init_adj.mp_transition = NULL;  
    path.push_back(init_adj);

    Adjacency current_adj = init_adj;
    for( unsigned int j = 0; j < str_id.size(); j ++ ) {
      string id = str_id[j];

      Adjacency next_adj = current_adj.mp_state->findAdjacency( id );
      path.push_back( next_adj );
      current_adj = next_adj;
    }
    path_list.push_back( path );
  }
  return path_list;
}

} // homotopy

} // topologyPathPlanning
