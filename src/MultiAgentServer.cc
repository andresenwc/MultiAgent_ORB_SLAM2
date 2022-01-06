#include "MultiAgentServer.h"

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

    // Create the Atlas
    mpMap = new Map();

    // Create Drawers. These are used by the Viewer.
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    // Initialize the Multi Agent Loop Closing thread and launch
    mpLoopClosing = new MultiAgentLoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=ORB_SLAM2::MONOCULAR);
    mptLoopClosing = new thread(&MultiAgentLoopClosing::Run, mpLoopClosing);

    // Initialize the Viewer thread and launch
    mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer,
                              mSensor, string(strSettingsFile), string("Server"));
    mptViewer = new thread(&Viewer::Run, mpViewer);
}

void MultiAgentServer::RegisterClient(System* client) {
    clients.push_back(client);
}

}
