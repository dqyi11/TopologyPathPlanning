#ifndef MLRRTSTAR_H
#define MLRRTSTAR_H

#include <vector>
#include <list>
#include "kdtreeml2d.h"
#include "expanding_tree_mgr.h"
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
    std::list<MLRRTNode*> m_child_nodes;
    std::vector< std::string > m_substring;

    ExpandingNode* mp_master;
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

    void init_feasible_paths();
 
    void extend();
    std::vector<Path*> get_paths();
    Path* _get_path( StringClass* p_string_class );
    Path* _get_path( MLRRTNode* p_node ); 
    
    double _calculate_cost( POS2D& pos_a, POS2D& pos_b );
    int**& get_map_info() { return _pp_map_info; }
    std::list<MLRRTNode*>& get_nodes() { return _nodes; }

    void set_reference_frames( homotopy::ReferenceFrameSet* p_reference_frames );
    homotopy::ReferenceFrameSet* get_reference_frames() { return _reference_frames; }
    ExpandingTreeMgr* get_expanding_tree_mgr() { return _p_expanding_tree_mgr; }
    void set_grammar_type( homotopy::grammar_type_t type ) { _grammar_type = type; }
    homotopy::grammar_type_t get_grammar_type() { return _grammar_type; }

    bool in_current_and_parent_exp_node( POS2D pos, ExpandingNode* p_exp_node );

  protected:
    POS2D _sampling();
    POS2D _steer( POS2D pos_a, POS2D pos_b );
    
    bool _is_obstacle_free( POS2D pos_a, POS2D pos_b );
    bool _is_in_obstacle( POS2D pos );
    bool _contains( POS2D pos );
    bool _is_homotopic_constrained( POS2D pos_a, POS2D pos_b, ExpandingNode* p_exp_node );
    
    KDNode2D _find_nearest( POS2D pos, ExpandingNode* p_exp_node );
    std::list<KDNode2D> _find_near( POS2D pos, ExpandingNode* p_exp_node );    

    MLRRTNode* _create_new_node( POS2D pos, ExpandingNode* p_exp_node );
    bool _attach_new_node( MLRRTNode* p_node_new, std::list<MLRRTNode*> near_nodes, ExpandingNode* p_exp_node );
    void _rewire_near_nodes( MLRRTNode* p_node_new, std::list<MLRRTNode*> near_nodes, ExpandingNode* p_exp_node );  

    bool _has_edge( MLRRTNode* p_node_parent, MLRRTNode* p_node_child );
    bool _add_edge( MLRRTNode* p_node_parent, MLRRTNode* p_node_child );
    bool _remove_edge( MLRRTNode* p_node_parent, MLRRTNode* p_node_child );

    POS2D      _start;
    POS2D      _goal;
    MLRRTNode* _p_root; 
  
    homotopy::grammar_type_t _grammar_type;

    int _sampling_width;
    int _sampling_height;

    int**                        _pp_map_info;
    homotopy::ReferenceFrameSet* _reference_frames;
    homotopy::StringGrammar*     _string_grammar;
    ExpandingTreeMgr*            _p_expanding_tree_mgr;
    KDTree2D*                    _p_master_kd_tree;
    COST_FUNC_PTR                _p_cost_func;
    double**                     _pp_cost_distribution;  
 
    std::list<MLRRTNode*>        _nodes;

    double _range; 
    double _segment_length;
    int    _obs_check_resolution;

    double _theta;
    int    _current_iteration;
  };

  inline MLRRTNode* get_ancestor( MLRRTNode * node ) {
    if( NULL == node ) {
      return NULL;
    }
    if( NULL == node->mp_parent ) {
      return node;
    }
    else {
      return get_ancestor( node->mp_parent );
    }
  }

  inline void get_parent_node_list( MLRRTNode * node, std::list<MLRRTNode*>& path ) {
    if( node==NULL ) {
      return;
    }
    path.push_back( node );
    get_parent_node_list( node->mp_parent, path );
    return;
  }
}


#endif /* MLRRTSTAR_H */
