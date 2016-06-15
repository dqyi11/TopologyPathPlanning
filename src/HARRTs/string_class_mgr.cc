#include <limits>
#include "string_class_mgr.h"
#include "birrtstar.h"

using namespace homotopy;
using namespace birrts;

StringClass::StringClass( std::vector< std::string > string, unsigned int created_iteration_num ) {
  m_string = string;
  m_cost = std::numeric_limits<float>::max();
  mp_path = NULL;
  m_created_iteration_num = created_iteration_num;
}

StringClass::~StringClass() {
  m_string.clear();
  m_historical_data.clear();
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

void StringClass::dump_historical_data( std::string filename ) {
  std::ofstream hist_data_file;
  hist_data_file.open(filename.c_str());
  hist_data_file << get_name() << std::endl;
  write_historical_data( hist_data_file );
  hist_data_file.close();
}

void StringClass::write_historical_data( std::ostream& out ) {
  for(unsigned int i=0;i<m_created_iteration_num;i++) {
    out << std::numeric_limits<float>::max() << " ";
  }
  //out << std::endl;
  for(std::vector<double>::iterator it = m_historical_data.begin();
      it != m_historical_data.end(); it++ ) {
    double data = (*it);
    out << data << " ";
  }
  out << std::endl;
}

void StringClass::record() {

  m_historical_data.push_back(m_cost);
}

StringClassMgr::StringClassMgr( StringGrammar* p_grammar ) {
  _p_grammar = p_grammar;
}

StringClassMgr::~StringClassMgr() {
  _p_grammar = NULL;
  _classes.clear();
}

void StringClassMgr::import_path( Path* p_path, unsigned int iteration_num ) { 
  std::vector< std::string > non_repeating_id_string = _p_grammar->get_non_repeating_form( p_path->m_string );
  if ( _p_grammar->is_valid_string( non_repeating_id_string ) == false ) {
    std::cout << "INVALID STRING " << std::endl;
  }
  StringClass* p_string_class = find_string_class( non_repeating_id_string );
  if( p_string_class ) {
    if( p_string_class->m_cost > p_path->m_cost ) {
      p_string_class->m_cost = p_path->m_cost;
      p_string_class->mp_path = p_path;      
    }
  }
  else {
    p_string_class = new StringClass( non_repeating_id_string, iteration_num ); 
    p_string_class->m_cost = p_path->m_cost;
    p_string_class->mp_path = p_path;
    _classes.push_back(p_string_class);
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

void StringClassMgr::merge() {
  std::vector< StringClass* > merged_classes;
  //std::cout << "NUM OF CLASSES " << _classes.size() << std::endl;
  for( unsigned int i = 0; i < _classes.size(); i ++ ) {
    StringClass* str_class = _classes[i];
    if( merged_classes.size() == 0 ) {
      merged_classes.push_back( str_class );
    }
    else {
      //std::cout << "MERGE CLASS SIZE " << merged_classes.size() << std::endl;
      bool found_equivalence = false;
      for( unsigned int j = 0; j < merged_classes.size(); j++) {
        StringClass* str_class_in_mer = merged_classes[j];
        //std::cout << "COMPARE [" << str_class->get_name() << "] and [" << str_class_in_mer->get_name() << "]" << std::endl;
        if( _p_grammar->is_equivalent( str_class_in_mer->m_string, str_class->m_string ) ) {
          if( str_class_in_mer->m_string.size() < str_class->m_string.size() ) {
            str_class_in_mer->m_string = str_class->m_string;
          }
          if ( str_class_in_mer->m_cost > str_class->m_cost ) {
            str_class_in_mer->m_cost = str_class->m_cost;
            str_class_in_mer->mp_path = str_class->mp_path;
          }
          found_equivalence = true;
          break;
        }
      }
      if( found_equivalence == false) {
          merged_classes.push_back( str_class );
      }
    }
  }

  _classes = merged_classes;
}
  
void StringClassMgr::export_grammar( std::string filename ) {
  if( _p_grammar ) {
    _p_grammar->output( filename );
  }
}

void StringClassMgr::dump_historical_data( std::string filename ) {
  std::ofstream hist_data_file;
  hist_data_file.open(filename.c_str());
  for(std::vector< StringClass* >::iterator it = _classes.begin();
      it != _classes.end(); it++ ) {
    StringClass* p_str_cls = (*it);
    if(p_str_cls) {
      hist_data_file << p_str_cls->get_name() << " : ";
      p_str_cls->write_historical_data( hist_data_file );
    }
  }
  hist_data_file.close();
}

void StringClassMgr::record() {

  for(std::vector< StringClass* >::iterator it = _classes.begin();
      it != _classes.end(); it++ ) {
    StringClass* p_str_cls = (*it);
    if(p_str_cls) {
      p_str_cls->record();
    }
  }
}
