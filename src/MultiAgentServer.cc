#include "MultiMap.h"
#include "MapFusion.h"

namespace ORB_SLAM2 {

MultiAgentServer::MultiAgentServer(
    const string &strVocFile, const string &strSettingsFile, const int sensor
): mSensor(sensor)
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

    // Create MultiMapDrawer. This is used by the ServerViewer.
    mpMultiMapDrawer = new MultiMapDrawer(mpMultiMap, strSettingsFile);

    // Initialize the Map Fusion thread and launch
    mpMapFusion = new MapFusion(this, mpMultiMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=ORB_SLAM2::MONOCULAR);
    mptMapFusion = new thread(&MapFusion::Run, mpMapFusion);

    // Initialize the Viewer thread and launch
    mpServerViewer = new ServerViewer(this, mpMultiMapDrawer, mSensor, string(strSettingsFile));
    mptServerViewer = new thread(&ServerViewer::Run, mpServerViewer);
}

void MultiAgentServer::RegisterClient(System* client) {
    clients.push_back(client);
}

void MultiAgentServer::InsertKeyFrame(KeyFrame *pKF) {
    mpMapFusion->InsertKeyFrame(pKF);
}

void MultiAgentServer::RequestStopMapping() {

    cout << "\tStopping local mapping." << endl;

    // request stops
    for (auto client : clients) {
        client->getLocalMapper()->RequestStop();
    }

    // wait for all local mappers to stop
    for (auto client : clients) {
        while(!(client->getLocalMapper()->isStopped())) {
            usleep(1000);
        }
    }

    cout << "\tLocal mapping stopped." << endl;
}

void MultiAgentServer::RequestReleaseMapping() {
    for (auto client : clients) {
        client->getLocalMapper()->Release();
    }

    cout << "\tLocal mapping released." << endl;
}

void MultiAgentServer::InformNewBigChange() {
    for (auto client : clients) {
        client->InformNewBigChange();
    }
}

}
