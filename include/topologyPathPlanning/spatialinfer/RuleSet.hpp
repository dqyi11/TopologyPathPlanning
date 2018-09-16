#ifndef TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_RULESET_HPP
#define TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_RULESET_HPP

#include "topologyPathPlanning/spatialinfer/Rule.hpp"

namespace topologyPathPlanning {

namespace topologyinference {
  
  class RuleSet : public Rule {
  public:
    RuleSet();
    virtual ~RuleSet();
  
    std::vector< Rule* > mSet;

  };

} // topologyinference

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_RULESET_HPP
