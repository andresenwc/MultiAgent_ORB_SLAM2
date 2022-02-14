
#ifndef SERVERVIEWER_H
#define SERVERVIEWER_H

#include "MultiAgentServer.h"
#include "MultiMapDrawer.h"

#include <mutex>

namespace ORB_SLAM2
{

class MultiMapDrawer;
class MultiAgentServer;

class ServerViewer
{
public:
    ServerViewer(MultiAgentServer* pServer, MultiMapDrawer* pMultiMapDrawer, const int sensor, const string &strSettingPath, const string &id = string("Server"));

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

private:
    bool Stop();

    MultiAgentServer* mpServer;
    MultiMapDrawer* mpMultiMapDrawer;
    int sensorType;
    string id;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;
    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

};

} //namespace ORB_SLAM2


#endif // SERVERVIEWER_H
	

