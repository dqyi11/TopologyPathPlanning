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

    virtual std::vector< std::pair<homotopy::ReferenceFrame*, bool> > get_rules( homotopy::ReferenceFrameSet* p_reference_frame_set ); 
    virtual std::string get_name();

    homotopy::Obstacle* mp_obstacle;
  };

} // topologyinference

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_AVOIDRELATIONFUNC_HPP
