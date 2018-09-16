#include <string>
#include <vector>
#include "topologyPathPlanning/homotopy/WorldDatatype.hpp"

namespace topologyPathPlanning {

namespace homotopy {

bool load_map_info( std::string filename, int& width, int& height, std::vector< std::vector<Point2D> >& obstacles );

bool load_map_info( int** pp_obstacle, int width, int height, std::vector< std::vector<Point2D> >& obstacles );

} // homotopy

} // topologyPathPlanning
