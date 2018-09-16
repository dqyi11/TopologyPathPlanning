#ifndef TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_RULE_HPP
#define TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_RULE_HPP

#include <vector>
#include "topologyPathPlanning/homotopy/ReferenceFrames.hpp"

namespace topologyPathPlanning {

namespace topologyinference {

  class Rule{
  public:
    Rule();
    virtual ~Rule();
  
    std::vector< Rule* > mRuleSequence;
    homotopy::ReferenceFrame* mpReferenceFrame;
  };

} // topologyinference

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_RULE_HPP
