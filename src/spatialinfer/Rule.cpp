#include "topologyPathPlanning/spatialinfer/Rule.hpp"

namespace topologyPathPlanning {

namespace topologyinference {

Rule::Rule() {
  mp_reference_frame = NULL;
}

Rule::~Rule() {
  m_rule_sequence.clear();
  mp_reference_frame = NULL;
}

} // topologyinference

} // topologyPathPlanning
