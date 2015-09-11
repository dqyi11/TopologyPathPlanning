#ifndef STRING_CLASS_MGR_H
#define STRING_CLASS_MGR_H

#include <vector>
#include "string_grammar.h"

class Path;

class StringClass {
public:
  StringClass( std::vector< std::string > string );
  virtual ~StringClass();
 
  std::vector< std::string > m_string;
  double m_cost;
  Path*  mp_path;
};

class StringClassMgr {

public:
  StringClassMgr(StringGrammar* p_grammar);
  virtual ~StringClassMgr();

  void import_path( Path* p_path );
  std::vector<Path*> export_paths();
  StringClass* find_string_class( std::vector< std::string > str );
  std::vector< StringClass* >& get_string_classes() { return _classes; }  
protected:
  StringGrammar* _p_grammar;
  std::vector< StringClass* > _classes;
};

#endif /* STRING_CLASS_MGR_H */
