#ifndef HASIRRTSTAR_H
#define HASIRRTSTAR_H

#include "HARRTstar.h"

class HASiRRTstar : public HARRTstar {

public:
  HASiRRTstar( int width, int height, int segment_length );
  virtual ~HASiRRTstar();

  void extend();
};

#endif // HASIRRTSTAR_H
