#ifndef STRING_CLASS_MGR_H
#define STRING_CLASS_MGR_H

#include <vector>
#include "string_grammar.h"

namespace harrts {

  class Path;

  class StringClass {
  public:
    StringClass( std::vector< std::string > string );
    virtual ~StringClass();
    std::string get_name();
  
    std::vector< std::string > m_string;
    double m_cost;
    Path*  mp_path;
  };

  class StringClassMgr {
  public:
    StringClassMgr(homotopy::StringGrammar* p_grammar);
    virtual ~StringClassMgr();

    void import_path( Path* p_path );
    std::vector<Path*> export_paths();
    void merge();
    StringClass* find_string_class( std::vector< std::string > str );
    std::vector< StringClass* >& get_string_classes() { return _classes; }  
    void export_grammar( std::string filename );
  protected:
    homotopy::StringGrammar* _p_grammar;
    std::vector< StringClass* > _classes;
  };

}

#endif /* STRING_CLASS_MGR_H */
