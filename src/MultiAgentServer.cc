#include "MultiAgentServer.h"
#include "MultiMap.h"
#include "MapFusion.h"

namespace ORB_SLAM2 {

MultiAgentServer::MultiAgentServer(
    const string &strVocFile, const string &strSettingsFile, const int sensor
): mSensor(sensor), mbPause(false)
{
    //----
    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    // Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    // Create the Multi-Map (set containing multiple maps)
    mpMultiMap = new MultiMap();

    // Initialize the Map Fusion thread and launch
    mpMapFusion = new MapFusion(this, mpMultiMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=ORB_SLAM2::MONOCULAR);
    mptMapFusion = new thread(&MapFusion::Run, mpMapFusion);
}

void MultiAgentServer::Shutdown() {
    mpMapFusion->RequestFinish();
    while (!mpMapFusion->isFinished() || mpMapFusion->isRunningCD() || mpMapFusion->isRunningGBA()) {
        usleep(5000);
    }
}

void MultiAgentServer::ShutdownSystems() {
    // shutdown viewers
    for (auto pSystem : clients) {
        pSystem->GetViewer()->RequestFinish();
    }

    // wait for viewers to shutdown
    for (auto pSystem : clients) {
        while (!pSystem->GetViewer()->isFinished()) {
            usleep(5000);
        }
    }

    // shutdown localmappers and loopclosers
    for (auto pSystem : clients) {
        pSystem->getLocalMapper()->RequestFinish();
        pSystem->GetLoopCloser()->RequestFinish();
    }

    // wait for localmappers and loopclosers to shutdown
    for (auto pSystem : clients) {
        while (!pSystem->getLocalMapper()->isFinished()
            || !pSystem->GetLoopCloser()->isFinished()
            || pSystem->GetLoopCloser()->isRunningGBA()
            ) {
            usleep(5000);
        }
    }
}

void MultiAgentServer::RegisterClient(System* client) {
    clients.push_back(client);
    mpMultiMap->AddSystemAndMap(client, client->GetMap());
}

void MultiAgentServer::InsertKeyFrame(KeyFrame *pKF) {
    mpMapFusion->InsertKeyFrame(pKF);
}

void MultiAgentServer::DeleteKFsFromDB(vector<KeyFrame*> vpKFs) {
    for (auto pKFi : vpKFs) {
        mpKeyFrameDatabase->erase(pKFi);
    }
}

void MultiAgentServer::SetPause(bool bPause) {
    mbPause = bPause;
}

bool MultiAgentServer::Pause() {
    return mbPause;
}

// Paused mapping for all SLAM systems attached to a map.
void MultiAgentServer::RequestStopMapping(Map* pMap) {
    // Systems to stop
    set<System*> pSystems = mpMultiMap->GetSystems(pMap);

    // request stops
    for (auto pSystem : pSystems) {
        pSystem->getLocalMapper()->RequestStop();
    }

    // wait for all local mappers to stop
    for (auto pSystem : pSystems) {
        while(!(pSystem->getLocalMapper()->isStopped())) {
            usleep(1000);
        }
    }
}

// Resume mapping for all SLAM systems attached to a map.
void MultiAgentServer::RequestReleaseMapping(Map* pMap) {
    set<System*> pSystems = mpMultiMap->GetSystems(pMap);
    for (auto pSystem : pSystems) {
        pSystem->getLocalMapper()->Release();
    }
}

}
