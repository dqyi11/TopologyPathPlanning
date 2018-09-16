#ifndef TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_SPATIALRELATIONFUNC_HPP
#define TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_SPATIALRELATIONFUNC_HPP

#include <utility>
#include <vector>
#include <string>
#include "topologyPathPlanning/homotopy/ReferenceFrames.hpp"

namespace topologyPathPlanning {

namespace topologyinference {

  class SpatialRelationFunction {
  public:
    SpatialRelationFunction() {};
    virtual ~SpatialRelationFunction() {};

    virtual std::vector< std::pair<homotopy::ReferenceFrame*, bool> > getRules( homotopy::ReferenceFrameSet* p_reference_frame_set ) = 0; 
    virtual std::string getName() = 0;
  };

} // topologyinference

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_SPATIALRELATIONFUNC_HPP
