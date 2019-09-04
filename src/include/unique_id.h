#ifndef UNIQUE_ID_H
#define UNIQUE_ID_H

#include <stdint.h>
#include <stdlib.h>

using namespace std;

class uniqueID
{
public:

    uniqueID();
    static uint64_t getID();
};

#endif // UNIQUE_ID_H
