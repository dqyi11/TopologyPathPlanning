#ifndef MLRRTSTAR_H
#define MLRRTSTAR_H

#include <vector>
#include <list>
#include "kdtreeml2d.h"
#include "string_class_mgr.h"
#include "reference_frames.h"

namespace mlrrts {

  typedef double (*COST_FUNC_PTR)(POS2D, POS2D, double**, void*);

  class MLRRTNode {
  
  public:
    MLRRTNode( POS2D pos );
    bool operator==( const MLRRTNode &other );
    
    POS2D m_pos;
    std::vector<MLRRTNode*> m_child_nodes;
    MLRRTNode*              mp_parent; 
  };

  class MLRRTstar {
  
  public:
    MLRRTstar( int width, int height, int segment_length );
    virtual ~MLRRTstar();
   
    bool init( POS2D start, POS2D goal, COST_FUNC_PTR p_func, double** pp_cost_distribution, homotopy::grammar_type_t grammar_type );
 
  protected:
    POS2D _start;
    POS2D _goal;
    MLRRTNode* _p_root; 
  
    homotopy::grammar_type_t _grammar_type;

    int _sampling_width;
    int _smapling_height;

    int**                    _pp_map_info;
    birrts::StringClassMgr*  _p_string_class_mgr;
    KDTree2D*                _kd_tree;
    COST_FUNC_PTR            _p_cost_func;
    double**                 _pp_cost_distribution;  
 
    std::list<MLRRTNode*> _nodes;
 
    double _segment_length;
    int    _obs_check_resolution;

    double _theta;
    int    _current_iteration;

  };
}


#endif /* MLRRTSTAR_H */
