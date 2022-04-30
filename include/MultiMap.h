#ifndef MULTIMAP_H
#define MULTIMAP_H

#include "System.h"
#include "Map.h"

#include <set>
#include <map>

namespace ORB_SLAM2 {

class Map;
class System;

class MultiMap{
    public:
        MultiMap();

        void AddSystemAndMap(System* pSystem, Map* pMap);
        void UpdateSystemMapAssociations(Map* pCurrentMap, Map* pMatchedMap);
        std::set<System*> GetSystems(Map* pMap);
        Map* GetMap(System* pSystem);

    private:
        std::map<System*, Map*> mmSystemToMap;
        std::map<Map*, std::set<System*>> mmMapToSystems;
};

} //namespace ORB_SLAM2

#endif // MULTIMAP_H