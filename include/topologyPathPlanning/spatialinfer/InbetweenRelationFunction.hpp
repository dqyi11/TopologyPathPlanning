#ifndef TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_INBETWEENRELATIONFUNC_HPP
#define TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_INBETWEENRELATIONFUNC_HPP

#include "topologyPathPlanning/homotopy/Obstacle.hpp"
#include "topologyPathPlanning/spatialinfer/SpatialRelationFunction.hpp"

namespace topologyPathPlanning {

namespace topologyinference {

  class InBetweenRelationFunction : public SpatialRelationFunction {
  public:
    InBetweenRelationFunction();
    virtual ~InBetweenRelationFunction();

    virtual std::vector< std::pair< homotopy::ReferenceFrame*, bool> > getRules( homotopy::ReferenceFrameSet* p_reference_frame_set ); 
    virtual std::string getName();

    std::vector<homotopy::Obstacle*> mpObstacles;
  };

} // topologyinference

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_INBETWEENRELATIONFUNC_HPP
