#include "MultiMap.h"

#include <set>

namespace ORB_SLAM2 {

MultiMap::MultiMap() {

}

void MultiMap::AddSystemAndMap(System* pSystem, Map* pMap) {
    mmSystemToMap[pSystem] = pMap;
    mmMapToSystems[pMap].insert(pSystem);
}

// Update mmSystemToMap and mmMapToSystems maps for systems/maps associated
// with the map fusion. Also updates the actual Map* in each system.
void MultiMap::UpdateSystemMapAssociations(
    Map* pCurrentMap, Map* pMatchedMap, System* pMatchedSystem) {

    // Get the systems that need updating
    std::set<System*> pCurrentSystems = GetSystems(pCurrentMap);

    // Update the stored relations structures, as well as each system's map.
    for (auto pSystem : pCurrentSystems) {
        mmMapToSystems[pCurrentMap].erase(pSystem);
        mmMapToSystems[pMatchedMap].insert(pSystem);
        mmSystemToMap[pSystem] = pMatchedMap;
        pSystem->SetMap(pMatchedMap);

        // for viewing
        if (pSystem->GetViewer()) {
            pSystem->GetViewer()->RequestFinish();
            pSystem->SetViewer(pMatchedSystem->GetViewer());
            pSystem->SetMapDrawer(pMatchedSystem->GetMapDrawer());
        }
    }

    // Related to visualization
    pMatchedMap->SetIsMerged();
}

std::set<System*> MultiMap::GetSystems(Map* pMap) {
    return mmMapToSystems[pMap];
}

Map* MultiMap::GetMap(System* pSystem) {
    return mmSystemToMap[pSystem];
}

}