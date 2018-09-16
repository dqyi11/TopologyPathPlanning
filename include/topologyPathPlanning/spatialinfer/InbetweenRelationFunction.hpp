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

    virtual std::vector< std::pair< homotopy::ReferenceFrame*, bool> > get_rules( homotopy::ReferenceFrameSet* p_reference_frame_set ); 
    virtual std::string get_name();

    std::vector<homotopy::Obstacle*> mp_obstacles;
  };

} // topologyinference

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_INBETWEENRELATIONFUNC_HPP
