#ifndef STRING_CLASS_MGR_H
#define STRING_CLASS_MGR_H

#include <vector>
#include "string_grammar.h"

class StringClass {

  std::vector< std::string > m_string;
  double m_cost;
  Path   m_path;
}

class StringClassMgr {

public:
  StringClassMgr(StringGrammar* p_grammar);
  virtual ~StringClassMgr();

protected:
  StringGrammar* _p_grammar;
  std::vector< StringClass > _classes;
};

#endif /* STRING_CLASS_MGR_H */
