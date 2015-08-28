#include "string_grammar.h"

State::State( std::string name ) {
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

Transition* State::find_transition( std::string trans_name ) {
    for( std::vector<Transition*>::iterator it = m_transitions.begin();
         it != m_transitions.end(); it++ ) {
        Transition* p_ref_transition = (*it);
        if ( p_ref_transition->m_name == trans_name ) {
            return p_ref_transition;
        }
    }
    return NULL;
}

Transition::Transition( State* p_from , State* p_to , std::string name ) {
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

State* StringGrammar::find_state( std::string name ) {
    for( std::vector<State*>::iterator it = _states.begin();
         it != _states.end(); it++ ) {
        State* p_ref_state = (*it);
        if ( p_ref_state->m_name == name ) {
            return p_ref_state;
        }
    }
    return NULL;
}

Transition* StringGrammar::find_transition( std::string name ) {
    for( std::vector<Transition*>::iterator it = _transitions.begin();
         it != _transitions.end(); it++ ) {
        Transition* p_ref_transition = (*it);
        if (p_ref_transition->m_name == name) {
            return p_ref_transition;
        }
    }
    return NULL;
}


bool StringGrammar::has_transition( Transition* p_transition ) {
    for( std::vector<Transition*>::iterator it = _transitions.begin();
         it != _transitions.end(); it++ ) {
        Transition* p_ref_transition = (*it);
        if ((*p_ref_transition) == (*p_transition)) {
            return true;
        }
    }
    return false;
}

bool StringGrammar::has_state( State* p_state ) {
    for( std::vector<State*>::iterator it = _states.begin();
         it != _states.end(); it++ ) {
        State* p_ref_state = (*it);
        if ( (*p_ref_state)==(*p_state) ) {
            return true;
        }
    }
    return false;
}

bool StringGrammar::add_transition( std::string from_name, std::string to_name, std::string name ) {
    Transition* p_transition = find_transition( name );
    if ( p_transition ) {
        return false;
    }
    State* p_from_state = find_state( from_name );
    if ( p_from_state == NULL ) {
        p_from_state = new State( from_name );
    }
    State* p_to_state = find_state( to_name );
    if ( p_to_state ) {
        p_to_state = new State( to_name );
    }

    p_transition = new Transition( p_from_state, p_to_state, name );
    _transitions.push_back( p_transition );
    p_from_state->m_transitions.push_back( p_transition );
    return true;
}

bool StringGrammar::set_init( std::string name ) {
    State* p_state = find_state( name );
    if ( p_state == NULL ) {
        return false;
    }
    _init_state = p_state;
    return true;
}

bool StringGrammar::set_goal( std::string name ) {
    State* p_state = find_state( name );
    if ( p_state == NULL ) {
        return false;
    }
    _goal_state = p_state;
    return true;
}

bool StringGrammar::is_valid_substring( std::vector< std::string > substr ) {
   State* p_current_state = _init_state;
   for ( std::vector< std::string >::iterator it = substr.begin();
         it != substr.end(); it++ ) {
       std::string name = (*it);
       Transition* p_trans = p_current_state->find_transition( name );
       if ( p_trans == NULL ) {
           return false;
       }
       p_current_state = p_trans->mp_to_state;
   }
   return true;
}

bool StringGrammar::is_valid_string( std::vector< std::string > str ) {
    unsigned int str_num = str.size();
    State* p_current_state = _init_state;
    for ( unsigned int i=0; i < str_num; i++ ) {
        std::string name = str[i];
        Transition* p_trans = p_current_state->find_transition( name );
        if ( p_trans == NULL ) {
            return false;
        }
        if( i < str_num-1 ) {
            p_current_state = p_trans->mp_to_state;
        }
        else {
            if ( p_trans->mp_to_state == _goal_state ) {
                return true;
            }
        }
    }
    return false;
}

