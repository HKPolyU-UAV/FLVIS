#include "unique_id.h"

static uint64_t id_index=10000;

uniqueID::uniqueID()
{
    //id_index=10000;
}

uint64_t uniqueID::getID()
{
    return ++id_index;
}
