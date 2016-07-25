#ifndef HOMOTOPIC_GRAMMAR_H
#define HOMOTOPIC_GRAMMAR_H

#include "string_grammar.h"

namespace homotopy {

  class HomotopicGrammar : public StringGrammar {
  public:
    HomotopicGrammar();
    virtual ~HomotopicGrammar();

    //virtual bool is_equivalent( std::vector< std::string > str_a , std::vector< std::string > str_b );
  };

}

#endif // HOMOTOPIC_GRAMMAR_H
