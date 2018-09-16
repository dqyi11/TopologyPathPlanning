#include "topologyPathPlanning/spatialinfer/Rule.hpp"

namespace topologyPathPlanning {

namespace topologyinference {

Rule::Rule() {
  mpReferenceFrame = NULL;
}

Rule::~Rule() {
  mRuleSequence.clear();
  mpReferenceFrame = NULL;
}

} // topologyinference

} // topologyPathPlanning
