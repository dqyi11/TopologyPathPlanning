#ifndef STRING_CLASS_MGR_H
#define STRING_CLASS_MGR_H

#include <vector>
#include <iostream>
#include "tpp/homotopy/string_grammar.h"

namespace birrts {

  class Path;

  class StringClass {
  public:
    StringClass( std::vector< std::string > string, unsigned int created_iteration_num = 0 );
    virtual ~StringClass();
    std::string get_name();
  
    std::vector< std::string > m_string;
    
    std::vector< double > m_historical_data;
    unsigned int m_created_iteration_num;   
    double m_cost;
    Path*  mp_path;

    void dump_historical_data( std::string filename );
    void write_historical_data( std::ostream& out );
    void record(); 
  };

  class StringClassMgr {
  public:
    StringClassMgr(homotopy::StringGrammar* p_grammar);
    virtual ~StringClassMgr();

    void import_path( Path* p_path, unsigned int iteration_num );
    std::vector<Path*> export_paths();
    void merge();
    StringClass* find_string_class( std::vector< std::string > str );
    std::vector< StringClass* >& get_string_classes() { return _classes; }  
    void export_grammar( std::string filename );
    void record();

    void dump_historical_data( std::string filename );
  protected:
    homotopy::StringGrammar* _p_grammar;
    std::vector< StringClass* > _classes;
  };

}

#endif /* STRING_CLASS_MGR_H */
