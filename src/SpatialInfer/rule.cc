#include "tpp/spatial_infer/rule.h"

using namespace topology_inference;

Rule::Rule() {
  mp_reference_frame = NULL;
}

Rule::~Rule() {
  m_rule_sequence.clear();
  mp_reference_frame = NULL;
}
