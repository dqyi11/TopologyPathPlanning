#ifndef TOPOLOGYPATHPLANNING_REFERENCE_FRAMES_HPP
#define TOPOLOGYPATHPLANNING_REFERENCE_FRAMES_HPP

#include <string>
#include "topologyPathPlanning/homotopy/Worldmap.hpp"
#include "topologyPathPlanning/homotopy/StringGrammar.hpp"
#include "topologyPathPlanning/homotopy/HomotopicGrammar.hpp"

namespace topologyPathPlanning {

namespace homotopy {

  static std::string CENTER_POINT_ID_CHARACTER = "C";

  typedef enum {
    STRING_GRAMMAR_TYPE,
    HOMOTOPIC_GRAMMAR_TYPE
  } grammar_type_t;

  class ReferenceFrame {
  public:
    ReferenceFrame( LineSubSegment* p_subseg );
    virtual ~ReferenceFrame();

    bool isLineCrossed( Point2D pos_a, Point2D pos_b );
    std::string getName() { return mName; }

    std::string     mName;
    Segment2D       mSegment;
    bool            mConnectToCp;
    LineSubSegment* mpLineSubsegment;
    Point2D         mMidPoint;
  };

  class ReferenceFrameSet {

  public:
    ReferenceFrameSet();
    virtual ~ReferenceFrameSet();

    void init(int width, int height, std::vector< std::vector<Point2D> >& obstacles);
    StringGrammar* getStringGrammar( Point2D init, Point2D goal );
    HomotopicGrammar* getHomotopicGrammar( Point2D init, Point2D goal );

    StringGrammar* getStringGrammar( int init_x, int init_y, int goal_x, int goal_y );
    HomotopicGrammar* getHomotopicGrammar( int init_x, int init_y, int goal_x, int goal_y );
    
    std::vector< std::string > getString ( Point2D start, Point2D end, grammar_type_t type );
    std::vector< std::string > getString ( std::vector<Point2D> points, grammar_type_t type );
    std::vector< std::string > getString ( PointSequence& path, grammar_type_t type );
    std::vector<ReferenceFrame*>& getReferenceFrames() { return mReferenceFrames; }

    WorldMap* getWorldMap() { return mpWorldMap; }
    ReferenceFrame* getReferenceFrame( std::string name );
   
    void importStringConstraint( std::vector<Point2D> points, grammar_type_t type );
    bool isConstainedSubstring( std::vector< std::string > sub_str, bool reverse );
    std::vector< std::vector< std::string > >& getStringConstraint() { return mStringConstraint; }

  protected:
    StringGrammar* getStringGrammar( SubRegion* p_init, SubRegion* p_goal );
    HomotopicGrammar* getHomotopicGrammar( SubRegion* p_init, SubRegion* p_goal );
    bool isEligibleSubstring( std::vector< std::string > substring, std::vector< std::string > ref_str, bool reverse );
  
    WorldMap*                                 mpWorldMap;
    std::vector<ReferenceFrame*>              mReferenceFrames;
    std::vector< std::vector< std::string > > mStringConstraint;
  };

} // homotopy

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_REFERENCE_FRAMES_HPP
