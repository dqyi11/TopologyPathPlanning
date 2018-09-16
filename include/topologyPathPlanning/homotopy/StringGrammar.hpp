#ifndef TOPOLOGYPATHPLANNING_STRING_GRAMMAR_HPP
#define TOPOLOGYPATHPLANNING_STRING_GRAMMAR_HPP

#include <vector>
#include <string>

namespace topologyPathPlanning {

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
    Adjacency findAdjacency( std::string trans_name );

    std::string              mName;
    // std::vector<Transition*> m_transitions;
    std::vector<Adjacency>   mAdjacencies;
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

    State* findState( std::string name );
    Transition* findTransition( std::string name );
    int getStateIndex( std::string name );

    bool hasTransition( Transition* p_transition );
    bool hasState( State* p_state );

    bool addTransition( std::string from_name, std::string to_name, std::string name );

    bool setInit( std::string name );
    bool setGoal( std::string name );

    bool isValidString( std::vector< std::string > str );
    bool isValidSubstring( std::vector< std::string > substr );

    std::vector< std::vector< std::string > > getAllSimpleStrings();
    void output( std::string filename ); 
    std::vector< std::string > getNonRepeatingForm( std::vector< std::string > str );
    virtual bool isEquivalent( std::vector< std::string > str_a , std::vector< std::string > str_b );
    std::vector< std::vector< Adjacency > > findSimplePaths();
    std::vector< std::vector< Adjacency > > findPaths( std::vector< std::vector< std::string > > strs );
    std::vector< std::vector< std::string > > findSimpleStrings();

protected:
    std::vector<Transition*> mTransitions;
    std::vector<State*>      mStates;

    State*                   mpInitState;
    State*                   mpGoalState;
  };

} // homotopy

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_STRING_GRAMMAR_HPP
