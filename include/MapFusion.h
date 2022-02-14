/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPFUSION_H
#define MAPFUSION_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "MultiMap.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class MultiMap;
class KeyFrame;


class MapFusion
{
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    // typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
    //     Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;

public:

    MapFusion(MultiMap* pMultiMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);

    // Main function
    void Run(); 

    // Add KeyFrame to the queue
    void InsertKeyFrame(KeyFrame* pKF);

    // For handling reset requests
    void RequestReset();

    // For handling finish requests
    void RequestFinish();
    bool isFinished();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    bool CheckNewKeyFrames();

    bool DetectFusionCandidates();

    // For handling reset requests
    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    // For handling finish requests
    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    MultiMap* mpMultiMap;

    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    std::list<KeyFrame*> mlpFusionKeyFrameQueue;
    std::mutex mMutexFusionQueue;

    // Fusion candidate detection parameters
    float mnCovisibilityConsistencyTh;

    // Fusion candidate detection variables (place recognition)
    KeyFrame* mpCurrentKF;
    // KeyFrame* mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    // std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    // std::vector<MapPoint*> mvpCurrentMatchedPoints;
    // std::vector<MapPoint*> mvpLoopMapPoints;
    // cv::Mat mScw;
    // g2o::Sim3 mg2oScw;

};

} //namespace ORB_SLAM

#endif // MAPFUSION_H
