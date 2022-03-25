#ifndef MULTIAGENTSERVER_H
#define MULTIAGENTSERVER_H

#include<string>
#include<stdio.h>
#include<vector>
#include<thread>

#include "Defines.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
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

        void RequestStopMapping();
        void RequestReleaseMapping();

        void InformNewBigChange();

    private:
        int mSensor;

        ORBVocabulary* mpVocabulary;

        KeyFrameDatabase* mpKeyFrameDatabase;

        MultiMap* mpMultiMap;

        MapFusion* mpMapFusion;
        std::thread* mptMapFusion;

        std::vector<System*> clients;
};
} // namespace ORB_SLAM2

#endif // MULTIAGENTSERVER_H
