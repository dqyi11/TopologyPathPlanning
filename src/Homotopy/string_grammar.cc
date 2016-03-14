#include <fstream>
#include <list>
#include <vector>
#include <queue>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/iteration_macros.hpp>
#include "string_grammar.h"

using namespace std;
using namespace boost;
using namespace homotopy;

struct Vertex{ string name; };
struct Edge{ string name; double weight; };

typedef adjacency_list<vecS, vecS, undirectedS, Vertex, Edge> Graph;
typedef graph_traits<Graph>::vertex_descriptor vertex_t;
typedef graph_traits<Graph>::edge_descriptor   edge_t;


State::State( string name ) {
  m_name = name;
}

State::~State() {

}

bool State::operator==(const State& other) const {
  if ( m_name == other.m_name ) {
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

Adjacency State::find_adjacency( string trans_name ) {
  Adjacency find_adj;
  find_adj.mp_state = NULL;
  find_adj.mp_transition = NULL;

  for( vector<Adjacency>::iterator it = m_adjacencies.begin();
       it != m_adjacencies.end(); it++ ) {
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

State* StringGrammar::find_state( string name ) {
  for( vector<State*>::iterator it = _states.begin();
       it != _states.end(); it++ ) {
    State* p_ref_state = (*it);
    if ( p_ref_state->m_name == name ) {
      return p_ref_state;
    }
  }
  return NULL;
}
    
int StringGrammar::get_state_index( string name ) {
  for( unsigned int i = 0; i < _states.size(); i ++ ) {
    if( _states[i]->m_name == name ) {
      return i;
    }
  }
  return -1;
}

Transition* StringGrammar::find_transition( string name ) {
  for( vector<Transition*>::iterator it = _transitions.begin();
       it != _transitions.end(); it++ ) {
    Transition* p_ref_transition = (*it);
    if (p_ref_transition->m_name == name) {
      return p_ref_transition;
    }
  }
  return NULL;
}


bool StringGrammar::has_transition( Transition* p_transition ) {
  for( vector<Transition*>::iterator it = _transitions.begin();
       it != _transitions.end(); it++ ) {
    Transition* p_ref_transition = (*it);
    if ((*p_ref_transition) == (*p_transition)) {
      return true;
    }
  }
  return false;
}

bool StringGrammar::has_state( State* p_state ) {
  for( vector<State*>::iterator it = _states.begin();
       it != _states.end(); it++ ) {
    State* p_ref_state = (*it);
    if ( (*p_ref_state)==(*p_state) ) {
      return true;
    }
  }
  return false;
}

bool StringGrammar::add_transition( string from_name, string to_name, string name ) {
  Transition* p_transition = find_transition( name );
  if ( p_transition ) {
    return false;
  }
  State* p_from_state = find_state( from_name );
  if ( p_from_state == NULL ) {
    p_from_state = new State( from_name );
    _states.push_back(p_from_state);
  }
  State* p_to_state = find_state( to_name );
  if ( p_to_state == NULL ) {
    p_to_state = new State( to_name );
    _states.push_back(p_to_state);
  }

  p_transition = new Transition( p_from_state, p_to_state, name );
  _transitions.push_back( p_transition );
  //p_from_state->m_transitions.push_back( p_transition );
  Adjacency from_adjacency;
  from_adjacency.mp_transition = p_transition;
  from_adjacency.mp_state = p_to_state; 
  p_from_state->m_adjacencies.push_back( from_adjacency );
  //p_to_state->m_transitions.push_back( p_transition );
  Adjacency to_adjacency;
  to_adjacency.mp_transition = p_transition;
  to_adjacency.mp_state = p_from_state; 
  p_to_state->m_adjacencies.push_back( to_adjacency );
  return true;
}

bool StringGrammar::set_init( string name ) {
  State* p_state = find_state( name );
  if ( p_state == NULL ) {
    return false;
  }
  _p_init_state = p_state;
  return true;
}

bool StringGrammar::set_goal( string name ) {
  State* p_state = find_state( name );
  if ( p_state == NULL ) {
    return false;
  }
  _p_goal_state = p_state;
  return true;
}

bool StringGrammar::is_valid_substring( vector< string > substr ) {
  State* p_current_state = _p_init_state;
  for ( vector< string >::iterator it = substr.begin();
        it != substr.end(); it++ ) {
    string name = (*it);
    Adjacency adj = p_current_state->find_adjacency( name );
    if ( adj.mp_transition == NULL ) {
      return false;
    }
    p_current_state = adj.mp_state;
  }
  return true;
}

bool StringGrammar::is_valid_string( vector< string > str ) {
  unsigned int str_num = str.size();
  State* p_current_state = _p_init_state;
  for ( unsigned int i=0; i < str_num; i++ ) {
    string name = str[i];
    Adjacency adj = p_current_state->find_adjacency( name );
    if ( adj.mp_transition == NULL ) {
      return false;
    }
    if( i < str_num-1 ) {
      p_current_state = adj.mp_state;
    }
    else {
      if ( adj.mp_state == _p_goal_state ) {
        return true;
      }
    }
  }
  return false;
}

bool StringGrammar::is_equivalent( vector< string > str_a , vector< string > str_b ) {
  vector< string > str_a_stack;
  vector< string > str_b_stack;
  
  str_a_stack = get_non_repeating_form( str_a );
  str_b_stack = get_non_repeating_form( str_b );
  
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

vector< string > StringGrammar::get_non_repeating_form( vector< string > id_str ) {
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
    
  const unsigned int edge_num = _transitions.size();
  const unsigned int vertex_num = _states.size();

  //cout << "STRING_GRAMMR::EDGE_NUM::" << edge_num << endl;
  //cout << "STRING_GRAMMR::VERTEX_NUM::" << vertex_num << endl;
  
  Graph g; 
  vector<vertex_t> vs;
  for( unsigned int i=0; i < vertex_num; i++) {
    vertex_t vt =  add_vertex( g );
    g[vt].name = _states[i]->m_name;
    vs.push_back(vt);
  }
  //cout << "Finish VERTEX INIT " << vs.size() << endl;
  vector<edge_t> es;
  for( unsigned int i=0; i < edge_num; i ++) {
    bool b = false;
    edge_t et;
    Transition* p_trans = _transitions[i];
    int s_i = get_state_index( p_trans->mp_from_state->m_name );
    int g_i = get_state_index( p_trans->mp_to_state->m_name );
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

vector< vector< string > > StringGrammar::get_all_simple_strings( ) {
  vector< vector< string > > simple_strings;
  if( _p_init_state && _p_goal_state ) { 
    simple_strings = find_simple_strings();
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
    if(path[i].mp_state->m_name == node.mp_state->m_name) { 
      return false;
    }
  }
  return true;
}

void print_path( vector< Adjacency > path ) {
  cout << "PATH:";
  for(unsigned int i=0;i<path.size();i++) {
    cout << path[i].mp_state->m_name.c_str() << " ";
  }
  cout << endl;
}

vector< vector< string > > StringGrammar::find_simple_strings() {

  vector< vector< Adjacency > > path_list = find_simple_paths();
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

vector< vector< Adjacency > > StringGrammar::find_simple_paths() {

  vector< vector< Adjacency > > path_list;
  vector< Adjacency > path;
  Adjacency init_adj;
  init_adj.mp_state = _p_init_state;
  init_adj.mp_transition = NULL;  
  path.push_back(init_adj);

  queue< vector< Adjacency > > q;
  q.push(path);

  while(!q.empty()) {
    path = q.front();
    q.pop();

    Adjacency last_node_of_path = path[path.size()-1];
    if( last_node_of_path.mp_state->m_name == _p_goal_state->m_name ) {
      print_path(path);
      path_list.push_back(path);
    }
    State* p_state = last_node_of_path.mp_state;
    if( p_state ) {
      for(unsigned int i=0; i<p_state->m_adjacencies.size(); i++) {
        Adjacency adj = p_state->m_adjacencies[i];
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

vector< vector< Adjacency > > StringGrammar::find_paths( vector< vector< string > > strs ) {

  vector< vector< Adjacency > > path_list;
  for( unsigned int i = 0; i < strs.size(); i++ ) {
    vector< string > str_id = strs[i];
    vector< Adjacency > path;
    Adjacency init_adj;
    init_adj.mp_state = _p_init_state;
    init_adj.mp_transition = NULL;  
    path.push_back(init_adj);

    Adjacency current_adj = init_adj;
    for( unsigned int j = 0; j < str_id.size(); j ++ ) {
      string id = str_id[j];

      Adjacency next_adj = current_adj.mp_state->find_adjacency( id );
      path.push_back( next_adj );
      current_adj = next_adj;
    }
    path_list.push_back( path );
  }
  return path_list;
}
