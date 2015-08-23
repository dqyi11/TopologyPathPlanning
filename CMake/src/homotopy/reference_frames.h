#ifndef REFERENCE_FRAMES_H
#define REFERENCE_FRAMES_H

#include "worldmap.h"
#include "string_grammar.h"
#include "homotopic_grammar.h"

class ReferenceFrames {

public:
    ReferenceFrames( WorldMap* p_world_map );
    virtual ~ReferenceFrames();

    StringGrammar* get_string_grammar( SubRegion* p_init, SubRegion* p_goal );
    HomotopicGrammar* get_homotopic_grammar( SubRegion* p_init, SubRegion* p_goal );

    WorldMap* mp_world_map;

};

#endif // REFERENCE_FRAMES_H
