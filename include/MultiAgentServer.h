#ifndef MULTIAGENTSERVER_H
#define MULTIAGENTSERVER_H

#include<string>
#include<stdio.h>
#include<vector>
#include<thread>

#include "Defines.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ServerViewer.h"
#include "MultiMapDrawer.h"
#include "MultiMap.h"
#include "MapFusion.h"
#include "System.h"

namespace ORB_SLAM2 {

class ServerViewer;
class MultiMap;
class MapFusion;
class System;

class MultiAgentServer {
    public:
        MultiAgentServer(const string &strVocFile, const string &strSettingsFile, const int sensor);

        void RegisterClient(System* client);
        void InsertKeyFrame(KeyFrame *pKF);

    private:
        int mSensor;

        ORBVocabulary* mpVocabulary;

        KeyFrameDatabase* mpKeyFrameDatabase;

        MultiMap* mpMultiMap;

        MultiMapDrawer* mpMultiMapDrawer;

        MapFusion* mpMapFusion;
        std::thread* mptMapFusion;

        ServerViewer* mpServerViewer;
        std::thread* mptServerViewer;

        std::vector<System*> clients;

        std::list<KeyFrame*> mlNewKeyFrames;
};
} // namespace ORB_SLAM2

#endif // MULTIAGENTSERVER_H
