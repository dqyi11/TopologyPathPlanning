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
  
    std::vector< Rule* > m_rule_sequence;
    homotopy::ReferenceFrame* mp_reference_frame;
  };

} // topologyinference

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_RULE_HPP
