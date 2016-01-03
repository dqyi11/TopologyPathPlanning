#ifndef INBETWEEN_RELATION_FUNC_H_
#define INBETWEEN_RELATION_FUNC_H_

#include "obstacle.h"
#include "spatial_relation_function.h"

namespace topology_inference {

  typedef enum{
    SIDE_TYPE_UNKNOWN,
    SIDE_TYPE_LEFT,
    SIDE_TYPE_RIGHT,
    NUM_SIDE_TYPES
  } side_type_t;

  class SideOfRelationFunction : SpatialRelationFunction {
  public:
    SideOfRelationFunction( side_type_t side_type = SIDE_TYPE_UNKNOWN );
    virtual ~SideOfRelationFunction();

    void set_side_type( side_type_t side_type ) { m_type = side_type; }
    side_type_t get_side_type( void ) { return m_type; }

    virtual std::vector<homotopy::ReferenceFrame*> get_reference_frames( homotopy::ReferenceFrameSet* p_reference_frame_set ); 

    homotopy::Obstacle* mp_obstacle;
    side_type_t         m_type;
  };
}

#endif // INBETWEEN_RELATION_FUNC_H_
