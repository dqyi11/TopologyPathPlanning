#ifndef HARRTSTAR_H
#define HARRTSTAR_H

#include <vector>
#include <list>
#include "KDTree2D.h"
#include "reference_frames.h"
#include "string_class_mgr.h"


namespace harrts {

  typedef double (*COST_FUNC_PTR)(POS2D, POS2D, double**, void*);

  typedef enum{
    START_TREE_TYPE,
    GOAL_TREE_TYPE
  } RRTree_type_t;

  typedef enum{
    RUN_START_TREE_TYPE,
    RUN_GOAL_TREE_TYPE,
    RUN_BOTH_TREES_TYPE
  } RRTree_run_type_t;

  class RRTNode {

  public:
    RRTNode( POS2D pos );
    bool operator==( const RRTNode &other );
    void clear_string();
    void append_to_string( std::vector< std::string > ids );

    double   m_cost;
    RRTNode* mp_parent;
    POS2D    m_pos;
    std::list<RRTNode*> m_child_nodes;
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

  class HARRTstar {

  public:
    HARRTstar(int width, int height, int segment_length);
    virtual ~HARRTstar();

    bool init( POS2D start, POS2D goal, COST_FUNC_PTR p_func, double** pp_cost_distrinution, homotopy::grammar_type_t grammar_type );
    void load_map( int** pp_map );

    int get_sampling_width() { return _sampling_width; }
    int get_sampling_height() { return _sampling_height; }
    int get_current_iteration() { return _current_iteration; }

    std::list<RRTNode*>& get_st_nodes() { return _st_nodes; }
    std::list<RRTNode*>& get_gt_nodes() { return _gt_nodes; }

    int**& get_map_info() { return _pp_map_info; }
    double get_st_ball_radius() { return _st_ball_radius; }
    double get_gt_ball_radius() { return _gt_ball_radius; }

    void extend();
    Path* find_path( POS2D via_pos );
    std::vector<Path*> get_paths();

    void set_reference_frames( homotopy::ReferenceFrameSet* p_reference_frames );
    homotopy::ReferenceFrameSet* get_reference_frames() { return _reference_frames; }
    StringClassMgr* get_string_class_mgr() { return _p_string_class_mgr; }

    void set_run_type( RRTree_run_type_t tree_type ) { _run_type = tree_type; }
    RRTree_run_type_t get_run_type() { return _run_type; } 
    homotopy::grammar_type_t get_grammar_type() { return _grammar_type; }
    void dump_distribution(std::string filename);

  protected:
    POS2D _sampling();
    POS2D _steer( POS2D pos_a, POS2D pos_b );
    RRTNode* _extend(RRTree_type_t tree_type);

    Path* _concatenate_paths( Path* p_from_path, Path* p_to_path );
    Path* _get_subpath( RRTNode* p_end_node, RRTree_type_t tree_type );

    KDNode2D _find_nearest( POS2D pos, RRTree_type_t tree_type );
    std::list<KDNode2D> _find_near( POS2D pos, RRTree_type_t tree_type );

    bool _is_homotopy_eligible( RRTNode* p_node_parent, POS2D pos, RRTree_type_t tree_type );
    bool _is_obstacle_free( POS2D pos_a, POS2D pos_b );
    bool _is_in_obstacle( POS2D pos );
    bool _contains( POS2D pos );

    double _calculate_cost( POS2D& pos_a, POS2D& pos_b );

    RRTNode* _create_new_node( POS2D pos, RRTree_type_t tree_type );
    bool _remove_edge( RRTNode* p_node_parent, RRTNode* p_node_child );
    bool _has_edge( RRTNode* p_node_parent, RRTNode* p_node_child );
    bool _add_edge( RRTNode* p_node_parent, RRTNode* p_node_child );

    std::list<RRTNode*> _find_all_children( RRTNode* node );

    void _attach_new_node( RRTNode* p_node_new, RRTNode* p_nearest_node, std::list<RRTNode*> near_nodes, RRTree_type_t type );
    void _rewire_near_nodes( RRTNode* p_node_new, std::list<RRTNode*> near_nodes, RRTree_type_t tree_type );
    void _update_cost_to_children( RRTNode* p_node, double delta_cost );
    bool _get_closest_node( POS2D pos, RRTNode*& p_node_closest, double& delta_cost, RRTree_type_t tree_type );

    RRTNode* _find_ancestor( RRTNode* p_node );
    void set_grammar_type( homotopy::grammar_type_t grammar_type ) { _grammar_type = grammar_type; }

    homotopy::ReferenceFrameSet* _reference_frames;
    homotopy::StringGrammar*     _string_grammar;
 
    POS2D    _start;
    POS2D    _goal;
    RRTNode* _p_st_root;
    RRTNode* _p_gt_root;

    homotopy::grammar_type_t _grammar_type;

    RRTNode* _p_st_new_node;
    RRTNode* _p_gt_new_node;
    RRTNode* _p_st_connected_node;
    RRTNode* _p_gt_connected_node;

    int _sampling_width;
    int _sampling_height;

    int**           _pp_map_info;
    StringClassMgr* _p_string_class_mgr;
    KDTree2D*       _p_st_kd_tree;
    KDTree2D*       _p_gt_kd_tree;
    COST_FUNC_PTR   _p_cost_func;
    double**        _pp_cost_distribution;

    std::list<RRTNode*> _st_nodes;
    std::list<RRTNode*> _gt_nodes;

    RRTree_run_type_t _run_type;
    double _range;
    double _st_ball_radius;
    double _gt_ball_radius;
    double _segment_length;
    int    _obs_check_resolution;

    double _theta;
    int    _current_iteration;
  };

  inline RRTNode* get_ancestor( RRTNode * node ) {
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

  inline void get_parent_node_list( RRTNode * node, std::list<RRTNode*>& path ) {
    if( node==NULL ) {
      return;
    }
    path.push_back( node );
    get_parent_node_list( node->mp_parent, path );
    return;
  }
}

#endif // HARRTSTAR_H
