#include "MultiAgentServer.h"

namespace ORB_SLAM2 {

MultiAgentServer::MultiAgentServer(
    const string &strVocFile, const string &strSettingsFile, const int sensor
): sensorType(sensor)
{
    //----
    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    vocabulary = new ORBVocabulary();
    bool bVocLoad = vocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    // Create KeyFrame Database
    globalKeyFrameDatabase = new KeyFrameDatabase(*vocabulary);

    // Create the Atlas
    globalMap = new Map();

    // Create Drawers. These are used by the Viewer.
    globalFrameDrawer = new FrameDrawer(globalMap);
    globalMapDrawer = new MapDrawer(globalMap, strSettingsFile);

    // Initialize the Viewer thread and launch
    globalViewer = new Viewer(this, globalFrameDrawer, globalMapDrawer,
                              sensorType, string(strSettingsFile), string("Server"));
}
}
