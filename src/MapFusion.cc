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

#include "MapFusion.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include<mutex>
#include<thread>

#include<chrono>

namespace ORB_SLAM2
{

MapFusion::MapFusion(MultiAgentServer* pServer, MultiMap *pMultiMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale):
    mpServer(pServer), mbResetRequested(false), mbFinishRequested(false),
    mbFinished(true), mpMultiMap(pMultiMap), mpKeyFrameDB(pDB),
    mpORBVocabulary(pVoc), mbFixScale(bFixScale), mpMatchedKF(NULL),
    mbRunningGBA(false), mbFinishedGBA(true), mbStopGBA(false), mpThreadGBA(NULL)
    // mLastLoopKFid(0), 
    // mnFullBAIdx(0)
{
    mnCovisibilityConsistencyTh = 3;
}

void MapFusion::Run()
{
    mbFinished =false;

    while(1)
    {
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Detect fusion candidates and check covisibility consistency
            if(DetectFusionCandidates())
            {
                // Compute similarity transformation (sim3)
                if (ComputeSim3()) {

                    // Perform map fusion
                    FuseMaps();
                }
            }
        }       

        ResetIfRequested();

        if(CheckFinish())
            break;

        usleep(5000);
    }

    SetFinish();
}

void MapFusion::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFusionQueue);
    if(pKF->mnId!=0)
        mlpFusionKeyFrameQueue.push_back(pKF);
}

bool MapFusion::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexFusionQueue);
    return(!mlpFusionKeyFrameQueue.empty());
}

bool MapFusion::DetectFusionCandidates()
{
    // YO THIS IS BUSTED

    {
        unique_lock<mutex> lock(mMutexFusionQueue);
        mpCurrentKF = mlpFusionKeyFrameQueue.front();
        mlpFusionKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose fusion candidates to have a higher similarity than this
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;
    }

    // Query the database imposing the minimum score
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

    // Discard same-map fusion candidates
    Map* pCurrentMap = mpCurrentKF->GetMap();
    vpCandidateKFs.erase(
        std::remove_if(
            vpCandidateKFs.begin(), vpCandidateKFs.end(),       
            [pCurrentMap](KeyFrame* pCandidateKF) {
                return pCurrentMap == pCandidateKF->GetMap();
            }
        ), vpCandidateKFs.end());

    // If there are no fusion candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();
        mpCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the
    // loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a
    // keyframe
    // We must detect a consistent loop in several consecutive keyframes to
    // accept it
    mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];

        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    // Used in processing of future KeyFrames
    mvConsistentGroups = vCurrentConsistentGroups;


    // Add Current Keyframe to database
    mpKeyFrameDB->add(mpCurrentKF);

    if(mvpEnoughConsistentCandidates.empty())
    {
        mpCurrentKF->SetErase();
        return false;
    }
    else
    {
        cout << "Fusion candidates detected." << endl;
        return true;
    }

    mpCurrentKF->SetErase();
    return false;
}

bool MapFusion::ComputeSim3() {
    // For each consistent loop candidate we try to compute a Sim3

    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    for(int i=0; i<nInitialCandidates; i++)
    {
        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->SetNotErase();

        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }

        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);

        if(nmatches<20)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);
            pSolver->SetRansacParameters(0.99,20,300);
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;
    }

    bool bMatch = false;

    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();
                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);

                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);

                // If optimization is succesful stop ransacs and continue
                if(nInliers>=20)
                {
                    bMatch = true;
                    mpMatchedKF = pKF;
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);

                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    if(nTotalMatches>=40)
    {
        cout << "\tMatches confirmed and Sim3 calculated." << endl;
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    else
    {
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }
}

void MapFusion::FuseMaps() {
    cout << "\tFusing Maps!" << endl;

    /**********
    ** Setup **
    **********/

    // Pause local mapping while fusion is done
    mpServer->RequestStopMapping();

    // Abort GBA if running
    if (isRunningGBA()) {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        if (mpThreadGBA) {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
    }

    // Some vars that we will use
    Map* pCurrentMap = mpCurrentKF->GetMap();
    Map* pMatchedMap = mpMatchedKF->GetMap();

    std::vector<KeyFrame*> vpCurrentMapKFs = pCurrentMap->GetAllKeyFrames();
    std::vector<MapPoint*> vpCurrentMapMPs = pCurrentMap->GetAllMapPoints();

    std::vector<KeyFrame*> vpMatchedMapKFs = pMatchedMap->GetAllKeyFrames();
    std::vector<MapPoint*> vpMatchedMapMPs = pMatchedMap->GetAllMapPoints();

    // Add the raw KFs and MPs to the merge map
    {
        // TODO: Add map lock

        // Move KFs from current to matched
        for (auto pKFi : vpCurrentMapKFs) {
            pKFi->SetMap(pMatchedMap);
            pMatchedMap->AddKeyFrame(pKFi);
            pCurrentMap->EraseKeyFrame(pKFi);
        }

        // Move MPs from current to matched
        for (auto pMPi : vpCurrentMapMPs) {
            pMPi->SetMap(pMatchedMap);
            pMatchedMap->AddMapPoint(pMPi);
            pCurrentMap->EraseMapPoint(pMPi);
        }
    }

    /**************
    ** Map Merge **
    **************/

    // Update current KF connections
    mpCurrentKF->UpdateConnections();

    // Retrieve KFs connected to the current KF
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    // KeyFrameAndPose structs for poses before/after correction
    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;

    {
        // TODO: Add map lock

        // Initialize the current KF with the Sim3 correction from ComputeSim3()
        CorrectedSim3[mpCurrentKF] = mg2oScw;
    
        // Inverse of current KF pose, used for calculating transformation matrix
        // between some KF's pose and the current KF's pose.
        // Inverse pose of current relative to (current) world.
        cv::Mat Twc = mpCurrentKF->GetPoseInverse();

        // Complete CorrectedSim3 and NonCorrectedSim3 for all connected KFs
        for (auto pKFi : vpCurrentMapKFs) {
            
            // CALCULATE THE CORRECTED POSES

            // Pose of i relative to (current) world
            cv::Mat Tiw = pKFi->GetPose();

            g2o::Sim3 g2oCorrectedSiw;

            // Calculate corrected pose for pKFi
            if (pKFi != mpCurrentKF) {
                // Pose of i relative to current
                cv::Mat Tic = Tiw*Twc;
                // Rotation matrix of i relative to current
                cv::Mat Ric = Tic.rowRange(0, 3).colRange(0, 3);
                // Translation vector of i relative to current
                cv::Mat tic = Tic.rowRange(0, 3).col(3);

                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),
                    Converter::toVector3d(tic), 1.0);

                g2oCorrectedSiw = g2oSic*mg2oScw;
            }
            else {
                g2oCorrectedSiw = mg2oScw;
            }

            // Rotation matrix of i relative to (current) world
            cv::Mat Riw = Tiw.rowRange(0, 3).colRange(0, 3);
            // Translation vector of i relative to (current) world
            cv::Mat tiw = Tiw.rowRange(0, 3).col(3);

            // Sim3 to move from (current) world to i
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),
                Converter::toVector3d(tiw), 1.0);

            // if in local neighborhood, we want to store some information
            // for future use
            if (find(mvpCurrentConnectedKFs.begin(),
                     mvpCurrentConnectedKFs.end(),
                     pKFi)
                != mvpCurrentConnectedKFs.end()) {
                
                // Store corrected Sim3
                CorrectedSim3[pKFi] = g2oCorrectedSiw;

                // Store uncorrected Sim3
                NonCorrectedSim3[pKFi] = g2oSiw;
            }

            // APPLY THE CORRECTED POSES

            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            // Update MPs seen by this KF
            vector<MapPoint*> vpMPi = pKFi->GetMapPointMatches();
            for (auto pMPi : vpMPi) {
                // Check if MP is bad, or has already been
                // corrected by this merge
                if (!pMPi || pMPi->isBad()
                    || pMPi->mnCorrectedByKF == mpCurrentKF->mnId) {
                    continue;
                }

                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double, 3, 1> eigP3Dw =
                    Converter::toVector3d(P3Dw);
                Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
                    g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();
            }

            // Update the KF
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();

            eigt *= (1.0/s);

            // The corrected pose
            cv::Mat correctedTiw = Converter::toCvSE3(eigR, eigt);
            pKFi->SetPose(correctedTiw);

            // Update connections
            pKFi->UpdateConnections();
        }

        // Start Map Fusion
        // Update matched MPs and replace if duplicated
        for (size_t i = 0; i < mvpCurrentMatchedPoints.size(); i++) {
            if (mvpCurrentMatchedPoints[i]) {
                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);

                if (pCurMP) {
                    pCurMP->Replace(pLoopMP);
                }
                else {
                    mpCurrentKF->AddMapPoint(pLoopMP,i);
                    pLoopMP->AddObservation(mpCurrentKF,i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }
    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3);

    // After the MapPoint fusion, new links in the covisibility graph will
    // appear attaching both sides of the loop. We want to discover and identify
    // these new links.
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;

    for (auto pKFi : mvpCurrentConnectedKFs) {

        // Covisibles before merge
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        // Discover new connections
        pKFi->UpdateConnections();

        // Initialize the set with all covisible KFs
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();

        // Eliminate covisibility and neighborhood KFs from before the merge,
        // leaving a vector of only the covisibility connections established by
        // the merge
        for (auto pKFprev : vpPreviousNeighbors)
        {
            LoopConnections[pKFi].erase(pKFprev);
        }
        for (auto pKFcurr : mvpCurrentConnectedKFs)
        {
            LoopConnections[pKFi].erase(pKFcurr);
        }
    }

    /************
    ** Cleanup **
    ************/

    // Make the SLAM systems point to the same map.
    pCurrentMap->GetSystem()->SetMap(pMatchedMap);
    pMatchedMap->SetIsMerged();

    // Merge the loop closure KFDBs
    KeyFrameDatabase* pMatchedKFDB =
        pMatchedMap->GetSystem()->GetLoopCloser()->GetKFDB();
    pMatchedMap->GetSystem()->GetLoopCloser()->AddKFsToDB(vpCurrentMapKFs);
    pCurrentMap->GetSystem()->GetLoopCloser()->SetKFDB(pMatchedKFDB);

    // Inform server of large map changes
    mpServer->InformNewBigChange();

    // Release local mapping
    mpServer->RequestReleaseMapping();

    /***********************************
    ** Global Covisibility Correction **
    ***********************************/

    mpThreadGCC = new thread(&MapFusion::CovisibilityDiscovery,
        this, vpCurrentMapKFs, pMatchedMap, pMatchedKFDB);
}

void MapFusion::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);

    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));
        matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(mpMatchedKF->GetMap()->mMutexMapUpdate);
        const int nLP = mvpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if(pRep)
            {
                pRep->Replace(mvpLoopMapPoints[i]);
            }
        }
    }
}

void MapFusion::CovisibilityDiscovery(
    std::vector<KeyFrame*> vpCurrentMapKFs, Map* pMatchedMap,
    KeyFrameDatabase* pMatchedKFDB) {

    cout << "\tCovisibility Discovery!" << endl;
    cout << "\t\tStarting Correction!" << endl;

    // ORBmatcher for matching MPs between KFs
    ORBmatcher matcher(0.75, true);

    // Detect, find, and fuse MapPoints
    for (auto pCurKFi : vpCurrentMapKFs) {

        /***********************************
        ** Detect Covisibility Candidates **
        ***********************************/

        // Calculate minimum similarity score for the query
        const vector<KeyFrame*> vpConnectedKFs =
            pCurKFi->GetVectorCovisibleKeyFrames();
        const DBoW2::BowVector &CurrentBowVec = pCurKFi->mBowVec;
        float minScore = 1;
        for (auto pKFi : vpConnectedKFs) {
            // Skip bad KFs
            if (pKFi->isBad())
                continue;
            
            // Compute score, replace minScore if new min
            const DBoW2::BowVector &BowVec = pKFi->mBowVec;
            float score = mpORBVocabulary->score(CurrentBowVec, BowVec);
            if (score < minScore)
                minScore = score;
        }

        // Query the DB for covisbility candidates imposing the min score
        vector<KeyFrame*> vpCandidateKFs = pMatchedKFDB->
            DetectCovisibilityCandidates(pCurKFi, minScore, vpCurrentMapKFs);
        
        // If there are no candidates, continue
        if (vpCandidateKFs.empty())
            continue;

        /***********************************
        ** Narrow Covisibility Candidates **
        ***********************************/

        size_t nInitialCandidates = vpCandidateKFs.size();

        vector<vector<MapPoint*>> vvpMapPointMatches;
        vvpMapPointMatches.resize(nInitialCandidates);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nInitialCandidates);
        
        // SearchByBoW
        // ORB matches
        for (size_t i = 0; i < nInitialCandidates; i++) {
            KeyFrame* pKFi = vpCandidateKFs[i];

            if (pKFi->isBad()) {
                vbDiscarded[i] = true;
                continue;
            }

            int nmatches =
                matcher.SearchByBoW(pCurKFi, pKFi, vvpMapPointMatches[i]);
            
            // Discard matches with too few ORB matches
            if (nmatches < 15) {
                vbDiscarded[i] = true;
                continue;
            }
        }

        /******************************
        ** Attempt to Fuse MapPoints **
        ******************************/

       cout << "\t\tFused: ";

        // For each candidate KF attempt to match and fuse MPs in the
        // neighborhood of the candidate KF with MPs in the current KF
        for (size_t i = 0; i < nInitialCandidates; i++) {
            if (vbDiscarded[i])
                continue;
            
            KeyFrame* pKFi = vpCandidateKFs[i];
            vector<MapPoint*> vpCandidateMPs = vvpMapPointMatches[i];

            vector<KeyFrame*> vpCovisConnectedKFs =
                pKFi->GetVectorCovisibleKeyFrames();
            vpCovisConnectedKFs.push_back(pKFi);

            vector<MapPoint*> vpCovisMPs;

            for (auto pKFj : vpCovisConnectedKFs) {
                for (auto pMPj : pKFj->GetMapPointMatches()) {
                    vpCovisMPs.push_back(pMPj);
                }
            }

            int nmatches = matcher.Fuse(pCurKFi, vpCovisMPs);

            cout << ", " << nmatches;
        }

        cout << endl;

        /***********************
        ** Update Connections **
        ***********************/

        pCurKFi->UpdateConnections();
    }

    // Make sure all connections are updated
    for (auto pKFi : vpCurrentMapKFs) {
        pKFi->UpdateConnections();
    }

    cout << "\t\tCorrection completed!" << endl;
}

bool MapFusion::isRunningGBA() {
    unique_lock<std::mutex> lock(mMutexGBA);
    return mbRunningGBA;
}

bool MapFusion::isFinishedGBA() {
    unique_lock<std::mutex> lock(mMutexGBA);
    return mbFinishedGBA;
}

void MapFusion::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        usleep(5000);
    }
}

void MapFusion::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlpFusionKeyFrameQueue.clear();
        // mLastLoopKFid=0;
        mbResetRequested=false;
    }
}

void MapFusion::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool MapFusion::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void MapFusion::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool MapFusion::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
