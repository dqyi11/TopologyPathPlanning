#ifndef RULE_SET_H_
#define RULE_SET_H_

#include "rule.h"

namespace topology_inference {
  
  class RuleSet : public Rule {
  public:
    RuleSet();
    virtual ~RuleSet();
  
    std::vector< Rule* > m_set;

  };

}

#endif // RULE_SET_H_
