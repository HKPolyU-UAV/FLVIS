#include "unique_id.h"

static long id_index=10000;

uniqueID::uniqueID()
{
    //id_index=10000;
}

long uniqueID::getID()
{
    return ++id_index;
}
