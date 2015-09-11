#include "string_class_mgr.h"
#include "HARRTstar.h"

StringClass::StringClass( std::vector< std::string > string ) {
  m_string = string;
  m_cost = 0.0;
  mp_path = NULL;
}

StringClass::~StringClass() {
  m_string.clear();
  m_cost = 0.0;
  mp_path = NULL;
}


StringClassMgr::StringClassMgr( StringGrammar* p_grammar ) {
  _p_grammar = p_grammar;
}

StringClassMgr::~StringClassMgr() {
  _p_grammar = NULL;
  _classes.clear();
}

void StringClassMgr::import_path( Path* p_path ) {
  StringClass* p_string_class = find_string_class( p_path->m_string );
  if( p_string_class ) {
    if( p_string_class->m_cost > p_path->m_cost ) {
      p_string_class->m_cost = p_path->m_cost;
      p_string_class->mp_path = p_path;      
    }
  }
  else {
    p_string_class = new StringClass( p_path->m_string ); 
    p_string_class->m_cost = p_path->m_cost;
    p_string_class->mp_path = p_path;
  }
}

std::vector<Path*> StringClassMgr::export_paths() {

  std::vector<Path*> paths;
  for( unsigned int i = 0; i < _classes.size(); i++) {
    paths.push_back( _classes[i]->mp_path );
  }
  return paths;
}

StringClass* StringClassMgr::find_string_class( std::vector< std::string > str ) {
  
  StringClass* p_string_class = NULL;
  for( unsigned int i = 0; i < _classes.size(); i ++ ) {
    if( _classes[i]->m_string.size() == str.size()) {
      bool identical = true;
      for( unsigned int j = 0; j < _classes[i]->m_string.size(); j ++) {
        if( _classes[i]->m_string[j] != str[j] ) {
          identical = false;
          break;
        }
      }
      if( identical ) {
        return _classes[i];
      }
    }
  }
  return p_string_class;
}
