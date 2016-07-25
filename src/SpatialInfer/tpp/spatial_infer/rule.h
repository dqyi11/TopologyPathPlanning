#ifndef RULE_H_
#define RULE_H_

#include <vector>
#include "tpp/homotopy/reference_frames.h"

namespace topology_inference {

  class Rule{
  public:
    Rule();
    virtual ~Rule();
  
    std::vector< Rule* > m_rule_sequence;
    homotopy::ReferenceFrame* mp_reference_frame;
  };

}

#endif // RULE_H_
