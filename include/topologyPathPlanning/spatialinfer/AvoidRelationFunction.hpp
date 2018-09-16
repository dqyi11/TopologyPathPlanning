#ifndef TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_AVOIDRELATIONFUNC_HPP
#define TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_AVOIDRELATIONFUNC_HPP

#include "topologyPathPlanning/homotopy/Obstacle.hpp"
#include "topologyPathPlanning/spatialinfer/SpatialRelationFunction.hpp"

namespace topologyPathPlanning {

namespace topologyinference {

  class AvoidRelationFunction : public SpatialRelationFunction {
  public:
    AvoidRelationFunction();
    virtual ~AvoidRelationFunction();

    virtual std::vector< std::pair<homotopy::ReferenceFrame*, bool> > getRules( homotopy::ReferenceFrameSet* p_reference_frame_set );
    virtual std::string getName();

    homotopy::Obstacle* mpObstacle;
  };

} // topologyinference

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_AVOIDRELATIONFUNC_HPP
