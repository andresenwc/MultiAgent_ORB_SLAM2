#ifndef MULTIAGENTSERVER_H
#define MULTIAGENTSERVER_H

#include<string>
#include<stdio.h>
#include<vector>
#include<thread>

#include "Defines.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "MultiAgentLoopClosing.h"
#include "Viewer.h"
#include "System.h"

namespace ORB_SLAM2 {

class Viewer;
class FrameDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class MultiAgentLoopClosing;
class System;

class MultiAgentServer {
    public:
        MultiAgentServer(const string &strVocFile, const string &strSettingsFile, const int sensor);

        void RegisterClient(System* client);

    private:
        int mSensor;

        ORBVocabulary* mpVocabulary;

        KeyFrameDatabase* mpKeyFrameDatabase;

        Map* mpMap;

        FrameDrawer* mpFrameDrawer;
        MapDrawer* mpMapDrawer;

        MultiAgentLoopClosing* mpLoopClosing;
        std::thread* mptLoopClosing;

        Viewer* mpViewer;
        std::thread* mptViewer;

        std::vector<System*> clients;
};
} // namespace ORB_SLAM2

#endif // MULTIAGENTSERVER_H
