// Driver code for running two EuRoC sequences

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<getopt.h>

#include<opencv2/core/core.hpp>

#include<Defines.h>
#include<MultiAgentServer.h>
#include<System.h>

using namespace std;

void LoadImages(
    const string &data_path, const string &ts_path,
    vector<vector<pair<string, string>>> &vvpstrImagesStereo,
    vector<vector<double>> &vvnTimestamps);

int main(int argc, char **argv) {

    if (argc != 7) {
        cerr << endl << "Usage: ./euroc_two_seq path_to_vocabulary path_to_settings path_to_data_seq1 path_to_ts_seq1 path_to_data_seq2 path_to_ts_seq2" << endl;
        return 1;
    }

    string voc_path = string(argv[1]);
    string sett_path = string(argv[2]);
    string data_seq1_path = string(argv[3]);
    string ts_seq1_path = string(argv[4]);
    string data_seq2_path = string(argv[5]);
    string ts_seq2_path = string(argv[6]);

    size_t num_agents = 2;
    int sensor_type = ORB_SLAM2::STEREO;

    // vectors containing paths to images
    vector<vector<pair<string, string>>> vvpstrImagesStereo;
    vector<vector<double>> vvnTimestamps;

    LoadImages(data_seq1_path, ts_seq1_path, vvpstrImagesStereo, vvnTimestamps);
    LoadImages(data_seq2_path, ts_seq2_path, vvpstrImagesStereo, vvnTimestamps);

    // Create Server. Initializes server threads and gets ready to process
    // keyframs from client SLAM systems.
    ORB_SLAM2::MultiAgentServer Server(voc_path, sett_path, sensor_type);

    // Create Client SLAM systems. Initializes all SLAM threads and gets
    // ready to process frames from each client.
    vector<ORB_SLAM2::System*> SLAMSystems;
    for (size_t i = 0; i < num_agents; i++) {
        SLAMSystems.push_back(new ORB_SLAM2::System(voc_path, sett_path, sensor_type, string("SLAM"+to_string(i)), true));
    }

    // Register Clients with Server and vice versa
    for (auto SLAMSystem : SLAMSystems) {
        Server.RegisterClient(SLAMSystem);
        SLAMSystem->RegisterServer(&Server);
    }

    // Read rectification parameters for stereo_euroc
    cv::Mat M1l,M2l,M1r,M2r;
    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    int rows_l, cols_l, rows_r, cols_r;

    cv::FileStorage fsSettings(sett_path, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        cerr << "ERROR: Wrong path to settings file." << endl;
        return -1;
    }

    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;


    rows_l = fsSettings["LEFT.height"];
    cols_l = fsSettings["LEFT.width"];
    rows_r = fsSettings["RIGHT.height"];
    cols_r = fsSettings["RIGHT.width"];


    if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty()
        || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty()
        || rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0) {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }


    cv::initUndistortRectifyMap(
        K_l, D_l, R_l,
        P_l.rowRange(0,3).colRange(0,3),
        cv::Size(cols_l,rows_l),
        CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(
        K_r, D_r, R_r,
        P_r.rowRange(0,3).colRange(0,3),
        cv::Size(cols_r,rows_r),
        CV_32F,M1r,M2r);

    // stats first line
    if (MAP_FUSION_STATS) {
        ofstream of("stats.csv");
        of << "sim3[µs],mf[µs],ckf,cmp,mkf,mmp,cd[µs],cdsum,cdmean,cdstdev,cdmed,gba[µs]" << endl;
        of.close();
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;

    // Print number of images to console
    cout << endl << "-------" << endl;
    cout << "Start processing sequences..." << endl;
    cout << "Images in the:" << endl;
    for (size_t i = 0; i < vvnTimestamps.size(); i++) {
        cout << "  - " << SLAMSystems[i]->mId << " sequence: " << vvnTimestamps[i].size() << endl;
    }

    // Vectors for each loop iteration
    vector<pair<cv::Mat*, cv::Mat*>> vpmatImagesStereo;
    vector<double> vnTimeStamps;
    vpmatImagesStereo.reserve(num_agents);
    vnTimeStamps.reserve(num_agents);

    // Iterators to handle simultaneous movement across all vectors
    vector<vector<pair<string, string>>::iterator> imItsStereo;
    vector<vector<double>::iterator> tsIts;

    imItsStereo.reserve(num_agents);
    tsIts.reserve(num_agents);

    // Initialize mono and stereo image its
    for (size_t i = 0; i < num_agents; i++) {
        imItsStereo[i] = vvpstrImagesStereo[i].begin();
    }
    
    // Initialize timestamp its
    for (size_t i = 0; i < num_agents; i++) {
        tsIts[i] = vvnTimestamps[i].begin();
    }

    size_t breakCounter = 0;

    while (true) {

        if (Server.Pause()) {
            cout << "Paused. Press a key to continue..." << endl;
            cin.get();
            Server.SetPause(false);
        }

        for (size_t i = 0; i < num_agents; i++) {
            vector<pair<string, string>>::iterator imIt = imItsStereo[i];
            if (imIt != vvpstrImagesStereo[i].end()) {
                cv::Mat *imLeft = new cv::Mat();
                cv::Mat *imRight = new cv::Mat();

                *imLeft = cv::imread(imIt->first, CV_LOAD_IMAGE_UNCHANGED);
                *imRight = cv::imread(imIt->second, CV_LOAD_IMAGE_UNCHANGED);

                if ((*imLeft).empty() || (*imRight).empty()) {
                    cerr << endl << "Failed to load one of these images: "
                        << string(imIt->first) << " "
                        << string(imIt->second) << endl;
                    return 1;
                }

                cv::Mat *imLeftRect = new cv::Mat();
                cv::Mat *imRightRect = new cv::Mat();
                cv::remap(*imLeft, *imLeftRect,
                    M1l, M2l, cv::INTER_LINEAR);
                cv::remap(*imRight, *imRightRect,
                    M1r, M2r, cv::INTER_LINEAR);
                vpmatImagesStereo[i] = make_pair(imLeftRect, imRightRect);
            }

            if (tsIts[i] != vvnTimestamps[i].end()) {
                vnTimeStamps[i] = *tsIts[i];
            }
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM systems
        for (size_t i = 0; i < num_agents; i++) {
            if (tsIts[i] != vvnTimestamps[i].end()) {
                cv::Mat imLeft = *(vpmatImagesStereo[i].first);
                cv::Mat imRight = *(vpmatImagesStereo[i].second);
                double ts = vnTimeStamps[i];
                SLAMSystems[i]->TrackStereo(imLeft, imRight, ts);
            }
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
        
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack.push_back(ttrack);

        // Update iterators
        for (size_t i = 0; i < num_agents; i++) {
            if (tsIts[i] != vvnTimestamps[i].end()) {
                tsIts[i]++;
                imItsStereo[i]++;
            }
        }

        // break if all sequences are fully processed
        for (size_t i = 0; i < num_agents; i++)
            if (tsIts[i] == vvnTimestamps[i].end())
                breakCounter++;
                
        if (breakCounter == num_agents)
            break;
        else
            breakCounter = 0;

        // Wait before loading the next image
        double T = 0;
        for (size_t i = 0; i < num_agents; i++) {
            vector<double>::iterator tsIt = tsIts[i];
            if (tsIt != vvnTimestamps[i].end()) {
                double Ti;
                if (tsIt != (vvnTimestamps[i].end()-1))
                    Ti = *(tsIt+1) - *tsIt;
                else
                    Ti = *tsIt - *(tsIt-1);
                T = max(T, Ti);
            }
        }
        if ((T-ttrack)*1e6 > 1e6)
            cout << "bad" << endl;
        if (ttrack < T)
            usleep((T-ttrack)*1e6);

    }

    // cout << "Press any button to continue..." << endl;
    // cin.get();

    // Stop all SLAM systems
    Server.ShutdownSystems();

    // Stop server
    Server.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for (auto time : vTimesTrack)
        totaltime += time;
    // images total
    int nImages = 0;
    for (auto vTs : vvnTimestamps)
        nImages += vTs.size();
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    for (auto SLAMSystem : SLAMSystems) {
        SLAMSystem->SaveTrajectoryTUM(SLAMSystem->mId);
    }

    return 0;
}



void LoadImages(
    const string &data_path, const string &ts_path,
    vector<vector<pair<string, string>>> &vvpstrImagesStereo,
    vector<vector<double>> &vvnTimestamps) {

    // prefixes
    string strPrefixLeft = data_path + "/cam0/data/";
    string strPrefixRight = data_path + "/cam1/data/";

    // paths vectors
    vector<pair<string, string>> vpstrImagesStereo;
    vector<double> vnTimestamps;

    // complete the paths vectors
    int i = 0;
    stringstream ss;
    string ts_line;
    string RGB;
    string D;
    double t = -1;
    ifstream fTimes;
    fTimes.open(ts_path.c_str());
    while (!fTimes.eof()) {
        // get the data
        getline(fTimes, ts_line);

        if (!ts_line.empty()) {
            // complete vectors
            ss << ts_line;
            pair<string, string> p = make_pair(
                    strPrefixLeft + ss.str() + ".png",
                    strPrefixRight + ss.str() + ".png");
            vpstrImagesStereo.push_back(p);
            ss >> t;
            vnTimestamps.push_back(t/1e9);
            getline(fTimes, ts_line);
        }

        ss.str(std::string());
        ss.clear();
        t = -1;

        ++i;
    }

    vvpstrImagesStereo.push_back(vpstrImagesStereo);
    vvnTimestamps.push_back(vnTimestamps);
}
