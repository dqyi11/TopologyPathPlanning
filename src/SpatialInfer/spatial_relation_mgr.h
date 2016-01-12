#ifndef SPATIAL_RELATION_MGR_H
#define SPATIAL_RELATION_MGR_H

#include <vector>
#include "worldmap.h"
#include "spatial_relation_function.h"

namespace topology_inference {

  class StringClass {
  public:
    StringClass( std::vector< std::string > string );
    virtual ~StringClass();
  
    std::string get_name();
    void init( homotopy::ReferenceFrameSet* p_rfs );
    
    std::vector< std::string >               m_string;
    std::vector< homotopy::ReferenceFrame* > mp_reference_frames;
  };

  class SpatialRelationMgr {
  public:
    SpatialRelationMgr(homotopy::WorldMap* p_worldmap);
    virtual ~SpatialRelationMgr();

    std::vector< std::pair<homotopy::ReferenceFrame*, bool> > get_rules( homotopy::ReferenceFrameSet* p_reference_frame_set );
    std::vector< std::string > get_spatial_relation_function_names();
  
    bool has_spatial_relation_function( std::string name );
    void remove_spatial_relation_function( std::string name );
  
    void get_string_classes( homotopy::ReferenceFrameSet* p_rfs  );
 
    homotopy::WorldMap* get_world_map() {  return mp_worldmap; }

    std::vector< std::vector< std::string > > filter( std::vector< std::vector< std::string > > string_set, std::vector< std::pair< homotopy::ReferenceFrame*, bool > > rules );
    bool is_eligible( std::vector< std::string > string_item, std::vector< std::pair< homotopy::ReferenceFrame*, bool > > rules );
    bool is_eligible( std::vector< std::string > string_item, std::pair< homotopy::ReferenceFrame*, bool >  rule );

    std::vector<StringClass*>                                   mp_string_classes;
    std::vector< std::pair< homotopy::ReferenceFrame*, bool > > m_rules;

    std::vector<SpatialRelationFunction*>                       mp_functions; 
    homotopy::WorldMap*                                         mp_worldmap;
    int m_start_x;
    int m_start_y;
    int m_goal_x;
    int m_goal_y;
  };
}

#endif // SPATIAL_RELATION_MGR_H
