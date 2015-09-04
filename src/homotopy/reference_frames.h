#ifndef REFERENCE_FRAMES_H
#define REFERENCE_FRAMES_H

#include "worldmap.h"
#include "string_grammar.h"
#include "homotopic_grammar.h"

typedef enum {
    STRING_GRAMMAR_TYPE,
    HOMOTOPIC_GRAMMAR_TYPE
} grammar_type_t;

class ReferenceFrames {

public:
    ReferenceFrames( WorldMap* p_world_map );
    virtual ~ReferenceFrames();

    StringGrammar* get_string_grammar( SubRegion* p_init, SubRegion* p_goal );
    HomotopicGrammar* get_homotopic_grammar( SubRegion* p_init, SubRegion* p_goal );

    std::string get_character_id( Point2D start, Point2D end, grammar_type_t type );

private:
    WorldMap*         _p_world_map;
};

#endif // REFERENCE_FRAMES_H
