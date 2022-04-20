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

/**
 * Driver code for running a two-agent KITTI simulation. Splits a KITTI dataset in half, providing half of the data to each agent.
 */

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<Defines.h>
#include<MultiAgentServer.h>
#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageLeft1, vector<string> &vstrImageRight1,
                vector<string> &vstrImageLeft2, vector<string> &vstrImageRight2,
                vector<double> &vTimestamps1, vector<double> &vTimestamps2);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti_multiagent path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft1, vstrImageRight1,
                   vstrImageLeft2, vstrImageRight2;
    vector<double> vTimestamps1, vTimestamps2;
    LoadImages(string(argv[3]), vstrImageLeft1, vstrImageRight1, vstrImageLeft2, vstrImageRight2, vTimestamps1, vTimestamps2);

    const int nImages1 = vstrImageLeft1.size();
    const int nImages2 = vstrImageLeft2.size();
    const int nImages = nImages1 + nImages2;

    // Create Server system. Initializes server threads and gets ready to process keyframs from client SLAM systems.
    ORB_SLAM2::MultiAgentServer Server(argv[1], argv[2], ORB_SLAM2::STEREO);

    // Create Client SLAM systems. Initializes all system threads and gets ready to process frames from each client.
    ORB_SLAM2::System SLAM1(argv[1],argv[2],ORB_SLAM2::STEREO,string("SLAM1"),true);
    ORB_SLAM2::System SLAM2(argv[1],argv[2],ORB_SLAM2::STEREO,string("SLAM2"),true);

    // Register Clients with Server and vice versa
    cout << endl << "Registering Clients and Server with each other." << endl;
    Server.RegisterClient(&SLAM1);
    Server.RegisterClient(&SLAM2);
    SLAM1.RegisterServer(&Server);
    SLAM2.RegisterServer(&Server);
    cout << "Registration complete." << endl;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the:\tSLAM1 sequence: " << nImages1 << endl;   
    cout << "              \tSLAM2 sequence: " << nImages2 << endl << endl;  

    // Main loop
    cv::Mat imLeft1, imRight1, imLeft2, imRight2;
    double tframe1, tframe2;

    auto itL1 = vstrImageLeft1.begin();
    auto itR1 = vstrImageRight1.begin();
    auto itT1 = vTimestamps1.begin();

    auto itL2 = vstrImageLeft2.begin();
    auto itR2 = vstrImageRight2.begin();
    auto itT2 = vTimestamps2.begin();

    while (itL1 != vstrImageLeft1.end() || itL2 != vstrImageLeft2.end())
    {
        // Load SLAM1 images
        if (itL1 != vstrImageLeft1.end()) {
            imLeft1 = cv::imread(*itL1, CV_LOAD_IMAGE_UNCHANGED);
            imRight1 = cv::imread(*itR1, CV_LOAD_IMAGE_UNCHANGED);
            tframe1 = *itT1;
            if(imLeft1.empty() || imRight1.empty()) {
                cerr << endl << "Failed to load one of these images: "
                    << string(*itL1) << string(*itR1) << endl;
                return 1;
            }
        }
        // Load SLAM1 images
        if (itL2 != vstrImageLeft2.end()) {
            imLeft2 = cv::imread(*itL2, CV_LOAD_IMAGE_UNCHANGED);
            imRight2 = cv::imread(*itR2, CV_LOAD_IMAGE_UNCHANGED);
            tframe2 = *itT2;
            if(imLeft2.empty() || imRight2.empty()) {
                cerr << endl << "Failed to load one of these images: "
                    << string(*itL2) << string(*itR2) << endl;
                return 1;
            }
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM systems
        if (itL1 != vstrImageLeft1.end())
            SLAM1.TrackStereo(imLeft1, imRight1, tframe1);
        if (itL2 != vstrImageLeft2.end())
            SLAM2.TrackStereo(imLeft2, imRight2, tframe2);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif


        // Update SLAM1 iterators
        if (itL1 != vstrImageLeft1.end()) {
            ++itL1;
            ++itR1;
            ++itT1;
        }
        // Update SLAM1 iterators
        if (itL2 != vstrImageLeft2.end()) {
            ++itL2;
            ++itR2;
            ++itT2;
        }

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack.push_back(ttrack);

        // Wait to load the next frame
        double T = 0, T1 = 0, T2 = 0;

        if (itL1 != vstrImageLeft1.end()) {
            if (itT1 != vTimestamps1.end()-1)
                T1 = *(itT1+1) - tframe1;
            else
                T1 = tframe1 - *(itT1-1);
        }

        if (itL2 != vstrImageLeft2.end()) {
            if (itT2 != vTimestamps2.end()-1)
                T2 = *(itT2+1) - tframe2;
            else
                T2 = tframe2 - *(itT2-1);
        }

        T = max(T1, T2);

        if (ttrack < T)
            usleep((T-ttrack)*1e6);
    }

    cout << "Press any button to continue..." << endl;
    cin.get();

    // Stop all threads
    SLAM1.Shutdown();
    SLAM2.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for (auto time : vTimesTrack)
        totaltime += time;
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM1.SaveTrajectoryKITTI("CameraTrajectory1.txt");
    SLAM2.SaveTrajectoryKITTI("CameraTrajectory2.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageLeft1, vector<string> &vstrImageRight1,
                vector<string> &vstrImageLeft2, vector<string> &vstrImageRight2,
                vector<double> &vTimestamps1, vector<double> &vTimestamps2) {
    
    // get all raw timestamps
    vector<double> vTimestamps;
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    // prefixes
    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";
    // size 
    const int nTimes = vTimestamps.size();
    // get all image names (paths)
    vector<string> vstrImageLeft, vstrImageRight;
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);
    for (int i = 0; i < nTimes; i++) {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }

    // midpoint index
    const size_t half_size = vstrImageLeft.size() / 2;

    // For use with SLAM1
    vstrImageLeft1 = vector<string>
        (vstrImageLeft.begin(), vstrImageLeft.begin() + half_size);
    vstrImageRight1 = vector<string>
        (vstrImageRight.begin(), vstrImageRight.begin() + half_size);
    vTimestamps1 = vector<double>
        (vTimestamps.begin(), vTimestamps.begin() + half_size);

    // For use with SLAM2
    vstrImageLeft2 = vector<string>
        (vstrImageLeft.begin() + half_size, vstrImageLeft.end());
    vstrImageRight2 = vector<string>
        (vstrImageRight.begin() + half_size, vstrImageRight.end());
    vTimestamps2 = vector<double>
        (vTimestamps.begin() + half_size, vTimestamps.end());
}
