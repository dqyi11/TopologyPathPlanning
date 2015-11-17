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
    void clear_string();
    void append_to_string( std::vector< std::string > ids );

    double   m_cost;
    MLRRTNode* mp_parent;
    POS2D m_pos;
    std::vector<MLRRTNode*> m_child_nodes;
    std::vector< std::string > m_substring;
  };

  class Path {

  public:
    Path(POS2D start, POS2D goal);
    ~Path();
    void append_waypoints( std::vector<POS2D> waypoints, bool reverse = false );
    void append_substring( std::vector< std::string > ids, bool reverse = false );

    double m_cost;
    POS2D  m_start;
    POS2D  m_goal;
    std::vector<POS2D> m_way_points;
    std::vector< std::string > m_string;
  };

  class MLRRTstar {
  
  public:
    MLRRTstar( int width, int height, int segment_length );
    virtual ~MLRRTstar();
   
    bool init( POS2D start, POS2D goal, COST_FUNC_PTR p_func, double** pp_cost_distribution, homotopy::grammar_type_t grammar_type );
    void load_map( int** pp_map );

    int get_sampling_width() { return _sampling_width; }
    int get_sampling_height() { return _sampling_height; }
    int get_current_iteration() { return _current_iteration; }
 
    void extend();
    Path* find_path( POS2D via_pos );
    std::vector<Path*> get_paths();

    int**& get_map_info() { return _pp_map_info; }

    void set_reference_frames( homotopy::ReferenceFrameSet* p_reference_frames );
    homotopy::ReferenceFrameSet* get_reference_frames() { return _reference_frames; }
    birrts::StringClassMgr* get_string_class_mgr() { return _p_string_class_mgr; }


  protected:
    POS2D _start;
    POS2D _goal;
    MLRRTNode* _p_root; 
  
    homotopy::grammar_type_t _grammar_type;

    int _sampling_width;
    int _sampling_height;

    int**                    _pp_map_info;
    homotopy::ReferenceFrameSet* _reference_frames;
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
