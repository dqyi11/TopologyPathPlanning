#ifndef TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_SIDEOFRELATIONFUNC_HPP
#define TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_SIDEOFRELATIONFUNC_HPP

#include "topologyPathPlanning/homotopy/Obstacle.hpp"
#include "topologyPathPlanning/spatialinfer/SpatialRelationFunction.hpp"

namespace topologyPathPlanning {

namespace topologyinference {

  typedef enum{
    SIDE_TYPE_UNKNOWN,
    SIDE_TYPE_LEFT,
    SIDE_TYPE_RIGHT,
    SIDE_TYPE_TOP,
    SIDE_TYPE_BOTTOM,
    NUM_SIDE_TYPES
  } side_type_t;

  class SideOfRelationFunction : public SpatialRelationFunction {
  public:
    SideOfRelationFunction( side_type_t side_type = SIDE_TYPE_UNKNOWN );
    virtual ~SideOfRelationFunction();

    void setSideType( side_type_t side_type ) { m_type = side_type; }
    side_type_t getSideType( void ) { return m_type; }

    virtual std::vector< std::pair<homotopy::ReferenceFrame*, bool> > getRules( homotopy::ReferenceFrameSet* p_reference_frame_set ); 
    virtual std::string getName();

    homotopy::Obstacle* mp_obstacle;
    side_type_t         m_type;
  };

} // topologyinference

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_SIDEOFRELATIONFUNC_HPP
