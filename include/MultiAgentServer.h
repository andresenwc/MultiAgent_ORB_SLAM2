#ifndef MULTIAGENTSERVER_H
#define MULTIAGENTSERVER_H

#include<string>
#include<stdio.h>

#include "Defines.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Viewer.h"


namespace ORB_SLAM2 {

class Viewer;
class FrameDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;

class MultiAgentServer {
    public:
        MultiAgentServer(const string &strVocFile, const string &strSettingsFile, const int sensor);

    private:
        int sensorType;

        ORBVocabulary* vocabulary;

        KeyFrameDatabase* globalKeyFrameDatabase;

        Map* globalMap;

        FrameDrawer* globalFrameDrawer;
        MapDrawer* globalMapDrawer;

        Viewer* globalViewer;
};
} // namespace ORB_SLAM2

#endif // MULTIAGENTSERVER_H
