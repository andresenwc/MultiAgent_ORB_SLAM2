#ifndef MULTIMAPDRAWER_H
#define MULTIMAPDRAWER_H

#include"MultiMap.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include<pangolin/pangolin.h>

#include<mutex>

namespace ORB_SLAM2
{

class MultiMapDrawer
{
public:
    MultiMapDrawer(MultiMap* pMultiMap, const string &strSettingPath);

private:
    MultiMap* mpMultiMap;

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

};

} //namespace ORB_SLAM

#endif // MULTIMAPDRAWER_H
