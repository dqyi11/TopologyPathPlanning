#ifndef TOPOLOGYPATHPLANNING_HARRTS_STRINGCLASSMGR_HPP
#define TOPOLOGYPATHPLANNING_HARRTS_STRINGCLASSMGR_HPP

#include <vector>
#include <iostream>
#include "topologyPathPlanning/homotopy/StringGrammar.hpp"

namespace topologyPathPlanning {

namespace harrts {

  class Path;

  class StringClass {
  public:
    StringClass( std::vector< std::string > string, unsigned int created_iteration_num = 0 );
    virtual ~StringClass();
    std::string getName();
  
    std::vector< std::string > mString;
    
    std::vector< double > mHistoricalData;
    unsigned int mCreatedIterationNum;
    double mCost;
    Path*  mpPath;

    void dump_historical_data( std::string filename );
    void write_historical_data( std::ostream& out );
    void record(); 
  };

  class StringClassMgr {
  public:
    StringClassMgr(homotopy::StringGrammar* p_grammar);
    virtual ~StringClassMgr();

    void importPath( Path* p_path, unsigned int iteration_num );
    std::vector<Path*> exportPaths();
    void merge();
    StringClass* findStringClass( std::vector< std::string > str );
    std::vector< StringClass* >& getStringClasses() { return mClasses; }
    void exportGrammar( std::string filename );
    void record();

    void dumpHistoricalData( std::string filename );
  protected:
    homotopy::StringGrammar* mpGrammar;
    std::vector< StringClass* > mClasses;
  };

} // harrts

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_HARRTS_STRINGCLASSMGR_HPP
