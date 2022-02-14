#include "MultiMap.h"

namespace ORB_SLAM2 {

MultiMap::MultiMap() {

}

void MultiMap::AddMap(Map* pMap) {
    msMaps.insert(pMap);
}

std::set<Map*> MultiMap::GetMaps() {
    return msMaps;
}

}