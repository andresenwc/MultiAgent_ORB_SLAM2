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

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft1,
                vector<string> &vstrImageRight1, vector<string> &vstrImageLeft2,
                vector<string> &vstrImageRight2, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti_multiagent path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft1, vstrImageRight1, vstrImageLeft2, vstrImageRight2;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageLeft1, vstrImageRight1, vstrImageLeft2, vstrImageRight2, vTimestamps);

    const int nImages = vstrImageLeft1.size();

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
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop
    cv::Mat imLeft1, imRight1, imLeft2, imRight2;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft1 = cv::imread(vstrImageLeft1[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight1 = cv::imread(vstrImageRight1[ni],CV_LOAD_IMAGE_UNCHANGED);
        imLeft2 = cv::imread(vstrImageLeft2[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight2 = cv::imread(vstrImageRight2[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft1.empty() || imRight1.empty())
        {
            cerr << endl << "Failed to load one of these images: "
                 << string(vstrImageLeft1[ni]) << string(vstrImageRight1[ni]) << endl;
            return 1;
        }
        if(imLeft2.empty() || imRight2.empty())
        {
            cerr << endl << "Failed to load one of these images: "
                 << string(vstrImageLeft2[ni]) << string(vstrImageRight2[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM systems
        SLAM1.TrackStereo(imLeft1,imRight1,tframe);
        SLAM2.TrackStereo(imLeft2,imRight2,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM1.Shutdown();
    SLAM2.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM1.SaveTrajectoryKITTI("CameraTrajectory1.txt");
    SLAM2.SaveTrajectoryKITTI("CameraTrajectory2.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft1,
                vector<string> &vstrImageRight1, vector<string> &vstrImageLeft2,
                vector<string> &vstrImageRight2, vector<double> &vTimestamps)
{
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

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    // last image is discarded if not even
    const int nTimes = vTimestamps.size() / 2;
    vstrImageLeft1.resize(nTimes);
    vstrImageRight1.resize(nTimes);
    vstrImageLeft2.resize(nTimes);
    vstrImageRight2.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss1, ss2;
        ss1 << setfill('0') << setw(6) << i;
        ss2 << setfill('0') << setw(6) << (i + nTimes);
        vstrImageLeft1[i] = strPrefixLeft + ss1.str() + ".png";
        vstrImageRight1[i] = strPrefixRight + ss1.str() + ".png";
        vstrImageLeft2[i] = strPrefixLeft + ss2.str() + ".png";
        vstrImageRight2[i] = strPrefixRight + ss2.str() + ".png";
    }
}
