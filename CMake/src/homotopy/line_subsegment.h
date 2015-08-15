#ifndef LINE_SUBSEGMENT_H
#define LINE_SUBSEGMENT_H

#include <vector>
#include "world_datatype.h"

class LineSubSegment {

public:
    LineSubSegment();
    ~LineSubSegment();

};

typedef enum {
    LINE_TYPE_UNKNOWN,
    LINE_TYPE_ALPHA,
    LINE_TYPE_BETA,
    NUM_LINE_TYPE
} line_subsegment_set_type_t;

class LineSubSegmentSet {

public:
    LineSubSegmentSet( Point2D& pos_a, Point2D& pos_b, unsigned int type, double rad );
    ~LineSubSegmentSet();

    bool load( std::vector<Point2D>& intersections );

    static std::string type_to_std_string ( const unsigned int& type );
    static unsigned int type_from_std_string ( const std::string& type_str );

    Segment2D    m_seg;
    double       m_rad;
    unsigned int m_type;
};

#endif // LINE_SUBSEGMENT_H
