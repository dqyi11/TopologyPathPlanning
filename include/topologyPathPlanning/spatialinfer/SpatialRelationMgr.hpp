#ifndef TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_SPATIALRELATIONMGR_HPP
#define TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_SPATIALRELATIONMGR_HPP

#include <vector>
#include "topologyPathPlanning/homotopy/Worldmap.hpp"
#include "topologyPathPlanning/spatialinfer/SpatialRelationFunction.hpp"

namespace topologyPathPlanning {

namespace topologyinference {

  class StringClass {
  public:
    StringClass( std::vector< std::string > string );
    virtual ~StringClass();
  
    std::string getName();
    void init( homotopy::ReferenceFrameSet* p_rfs );
    
    std::vector< std::string >               mString;
    std::vector< homotopy::ReferenceFrame* > mpReferenceFrames;
  };

  class SpatialRelationMgr {
  public:
    SpatialRelationMgr(homotopy::WorldMap* p_worldmap);
    virtual ~SpatialRelationMgr();

    std::vector< std::pair<homotopy::ReferenceFrame*, bool> > getRules( homotopy::ReferenceFrameSet* p_reference_frame_set );
    std::vector< std::string > getSpatialRelationFunctionNames();
  
    bool hasSpatialRelationFunction( std::string name );
    void removeSpatialRelationFunction( std::string name );
  
    void getStringClasses( homotopy::ReferenceFrameSet* p_rfs  );
 
    homotopy::WorldMap* getWorldMap() {  return mpWorldmap; }
    void printRules( std::vector< std::pair< homotopy::ReferenceFrame*, bool > > rules );

    std::vector< std::vector< std::string > > filter( std::vector< std::vector< std::string > > string_set, std::vector< std::pair< homotopy::ReferenceFrame*, bool > > rules );
    bool isEligible( std::vector< std::string > string_item, std::vector< std::pair< homotopy::ReferenceFrame*, bool > > rules );
    bool isEligible( std::vector< std::string > string_item, std::pair< homotopy::ReferenceFrame*, bool >  rule );

    std::vector<StringClass*>                                   mpStringClasses;
    std::vector< std::pair< homotopy::ReferenceFrame*, bool > > mRules;

    std::vector<SpatialRelationFunction*>                       mpFunctions;
    homotopy::WorldMap*                                         mpWorldmap;
    int mStartX;
    int mStartY;
    int mGoalX;
    int mGoalY;
  };

} // topologyinference

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_SPATIALRELATIONMGR_HPP
