#ifndef TOPOLOGYPATHPLANNING_HOMOTOPIC_GRAMMAR_HPP
#define TOPOLOGYPATHPLANNING_HOMOTOPIC_GRAMMAR_HPP

#include "topologyPathPlanning/homotopy/StringGrammar.hpp"

namespace topologyPathPlanning {

namespace homotopy {

  class HomotopicGrammar : public StringGrammar {
  public:
    HomotopicGrammar();
    virtual ~HomotopicGrammar();

    //virtual bool is_equivalent( std::vector< std::string > str_a , std::vector< std::string > str_b );
  };

} // homotopy

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_HOMOTOPIC_GRAMMAR_HPP
