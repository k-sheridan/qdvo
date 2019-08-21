#pragma once

#include "Types.h"

namespace QDVO
{

/**
 * A key class which allows for nested IDs.
 */
class Key
{
public:
    QDVO::ID inner, outer;

    Key(QDVO::ID outerKey, QDVO::ID innerKey) : inner(innerKey), outer(outerKey) {}

    bool operator<(const Key &comp) const
    {
        return outer < comp.outer || (outer == comp.outer && inner < comp.inner);
    }
};

} // namespace QDVO