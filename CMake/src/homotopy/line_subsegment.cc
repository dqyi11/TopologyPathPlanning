#include "line_subsegment.h"

LineSubSegmentSet::LineSubSegmentSet( Point2D& pos_a, Point2D& pos_b, unsigned int type, double rad ) {

    m_seg = Segment2D(pos_a, pos_b);
    m_type = type;
    m_rad = rad;
}

LineSubSegmentSet::~LineSubSegmentSet() {

}

std::string LineSubSegmentSet::type_to_std_string ( const unsigned int& type ) {
    switch( type ) {
    case( LINE_TYPE_ALPHA ):
        return "A";
    case( LINE_TYPE_BETA ):
        return "B";
    case( LINE_TYPE_UNKNOWN ):
    default:
        return "na";
    }
}

unsigned int LineSubSegmentSet::type_from_std_string ( const std::string& type_str ) {
    for( unsigned int i = 0; i < NUM_LINE_TYPE; i++ ){
      if( type_str == LineSubSegmentSet::type_to_std_string( i ) ){
        return i;
      }
    }
    return LINE_TYPE_UNKNOWN;
}
