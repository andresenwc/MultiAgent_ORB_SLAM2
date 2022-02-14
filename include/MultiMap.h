#ifndef MULTIMAP_H
#define MULTIMAP_H

#include "Map.h"

#include <set>

namespace ORB_SLAM2 {

class Map;

class MultiMap{
    public:
        MultiMap();

        void AddMap(Map* pMap);

        std::set<Map*> GetMaps();

    private:
        std::set<Map*> msMaps;
};

} //namespace ORB_SLAM2

#endif // MULTIMAP_H