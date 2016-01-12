#ifndef STRING_GRAMMAR_H
#define STRING_GRAMMAR_H

#include <vector>
#include <string>

namespace homotopy {

  class Transition;
  class State;

  struct Adjacency{
    Transition* mp_transition;
    State*      mp_state; 
  };

  class State {
  public:
    State( std::string name );
    virtual ~State();

    bool operator==(const State& other) const;

    //Transition* find_transition( std::string trans_name );
    Adjacency find_adjacency( std::string trans_name );

    std::string              m_name;
    // std::vector<Transition*> m_transitions;
    std::vector<Adjacency>   m_adjacencies;
  };

  class Transition {
  public:
    Transition( State* p_from , State* p_to , std::string name );
    virtual ~Transition();

    bool operator==(const Transition& other) const;

    std::string m_name;
    State*      mp_from_state;
    State*      mp_to_state;
  };

  class StringGrammar {
  public:    
    StringGrammar();
    virtual ~StringGrammar();

    State* find_state( std::string name );
    Transition* find_transition( std::string name );
    int get_state_index( std::string name );

    bool has_transition( Transition* p_transition );
    bool has_state( State* p_state );

    bool add_transition( std::string from_name, std::string to_name, std::string name );

    bool set_init( std::string name );
    bool set_goal( std::string name );

    bool is_valid_string( std::vector< std::string > str );
    bool is_valid_substring( std::vector< std::string > substr );

    std::vector< std::vector< std::string > > get_all_simple_strings();   
    void output( std::string filename ); 
    std::vector< std::string > get_non_repeating_form( std::vector< std::string > str );
    virtual bool is_equivalent( std::vector< std::string > str_a , std::vector< std::string > str_b );
    std::vector< std::vector< Adjacency > > find_simple_paths();
    std::vector< std::vector< Adjacency > > find_paths( std::vector< std::vector< std::string > > strs );
    std::vector< std::vector< std::string > > find_simple_strings();

protected:
    std::vector<Transition*> _transitions;
    std::vector<State*>      _states;

    State*                   _p_init_state;
    State*                   _p_goal_state;
  };
}

#endif // STRING_GRAMMAR_H
