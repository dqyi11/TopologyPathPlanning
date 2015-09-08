#ifndef HARRTSTAR_H
#define HARRTSTAR_H

#include <vector>
#include <list>
#include "KDTree2D.h"
#include "reference_frames.h"

typedef double (*COST_FUNC_PTR)(POS2D, POS2D, double**, void*);

typedef enum{
  START_TREE_TYPE,
  GOAL_TREE_TYPE
} RRTree_type_t;

class RRTNode {

public:
    RRTNode( POS2D pos );

    bool operator==( const RRTNode &other );

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

    double m_cost;
    POS2D  m_start;
    POS2D  m_goal;
    std::vector<POS2D> m_way_points;
    std::vector< std::string > m_string;
};

class HARRTstar {

public:
    HARRTstar(int width, int height, int segment_length);
    ~HARRTstar();

    void init( POS2D start, POS2D goal, COST_FUNC_PTR p_func, double** pp_cost_distrinution );

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
    Path* find_path();

    void set_reference_frames( ReferenceFrameSet* p_reference_frames ) { _reference_frames = p_reference_frames; }
    ReferenceFrameSet* get_reference_frames() { return _reference_frames; }

    void dump_distribution(std::string filename);

protected:
    POS2D _sampling();
    POS2D _steer( POS2D pos_a, POS2D pos_b );
    void extend(RRTree_type_t type);

    KDNode2D _find_nearest( POS2D pos, RRTree_type_t type );
    std::list<KDNode2D> _find_near( POS2D pos, RRTree_type_t type );

    bool _is_obstacle_free( POS2D pos_a, POS2D pos_b );
    bool _is_in_obstacle( POS2D pos );
    bool _contains( POS2D pos );

    double _calculate_cost( POS2D& pos_a, POS2D& pos_b );

    RRTNode* _create_new_node( POS2D pos, RRTree_type_t type );
    bool _remove_edge( RRTNode* p_node_parent, RRTNode* p_node_child );
    bool _has_edge( RRTNode* p_node_parent, RRTNode* p_node_child );
    bool _add_edge( RRTNode* p_node_parent, RRTNode* p_node_child );

    std::list<RRTNode*> _find_all_children( RRTNode* node );

    void _attach_new_node( RRTNode* p_node_new, RRTNode* p_nearest_node, std::list<RRTNode*> near_nodes );
    void _rewire_near_nodes( RRTNode* p_node_new, std::list<RRTNode*> near_nodes );
    void _update_cost_to_children( RRTNode* p_node, double delta_cost );
    bool _get_closet_to_goal( RRTNode*& p_node_closet_to_goal, double& delta_cost, RRTree_type_t type );

    RRTNode* _find_ancestor( RRTNode* p_node );

private:
    ReferenceFrameSet* _reference_frames;
 
    POS2D    _start;
    POS2D    _goal;
    RRTNode* _p_st_root;
    RRTNode* _p_gt_root;

    RRTNode* _p_st_new_node;
    RRTNode* _p_gt_new_node;
    RRTNode* _p_st_connected_node;
    RRTNode* _p_gt_connected_node;

    int _sampling_width;
    int _sampling_height;

    int** _pp_map_info;

    KDTree2D*     _p_st_kd_tree;
    KDTree2D*     _p_gt_kd_tree;
    COST_FUNC_PTR _p_cost_func;
    double**      _pp_cost_distribution;

    std::list<RRTNode*> _st_nodes;
    std::list<RRTNode*> _gt_nodes;

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

#endif // HARRTSTAR_H
