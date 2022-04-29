// Driver code for running multi-agent SLAM.
//
// Out of the box works with all ORB-SLAM2 test datasets (i.e. KITTI, EuRoC, 
// and TUM-RGBD).
// 
// Usage: ./stereo_kitti_multiagent [options]
//   Note that all of the following options (besides -h) are required.
//   Options:
//   -h                 Print this help message.
//   -t [seq_type]      The sequence type. Valid options: mono_kitti, mono_euroc,
//                      mono_tum, stereo_kitti, stereo_euroc, and rgbd_tum.
//   -n [num_agents]    The number of agents to use (2-4).
//   -v [voc_path]      The path to the ORB vocabulary.
//   -d [data_path]     The path to the data sequence.
//   -s [sett_path]     The path to the data sequence's settings yaml file.

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
    const string &data_path, const size_t &num_agents,
    const string &seq_type, const int &sensor_type,
    vector<vector<string>> &vvstrImagesMonocular,
    vector<vector<pair<string, string>>> &vvpstrImagesStereo,
    vector<vector<double>> &vvnTimestamps);

bool ParseArgs(int argc, char **argv,
    string &seq_type, size_t &num_agents, string &voc_path,
    string &data_path, string &sett_path, int &sensor_type);

void Usage();

int main(int argc, char **argv) {

    string seq_type, voc_path, data_path, sett_path;
    size_t num_agents = 0;
    int sensor_type = -1;

    if (!ParseArgs(argc, argv, seq_type, num_agents, voc_path, data_path, sett_path, sensor_type))
        return 0;
    else {
        cout << "args parsed succesfully" << endl;
    }

    // vectors containing paths to images
    vector<vector<string>> vvstrImagesMonocular;
    vector<vector<pair<string, string>>> vvpstrImagesStereo;
    vector<vector<double>> vvnTimestamps;

    LoadImages(
        data_path, num_agents,
        seq_type, sensor_type,
        vvstrImagesMonocular, vvpstrImagesStereo, vvnTimestamps);

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
    if (seq_type == "euroc" && sensor_type == ORB_SLAM2::STEREO) {
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
    vector<cv::Mat*> vmatImagesMonocular;
    vector<pair<cv::Mat*, cv::Mat*>> vpmatImagesStereo;
    vector<double> vnTimeStamps;
    vmatImagesMonocular.reserve(num_agents);
    vpmatImagesStereo.reserve(num_agents);
    vnTimeStamps.reserve(num_agents);

    // Iterators to handle simultaneous movement across all vectors
    vector<vector<string>::iterator> imItsMonocular;
    vector<vector<pair<string, string>>::iterator> imItsStereo;
    vector<vector<double>::iterator> tsIts;

    imItsMonocular.reserve(num_agents);
    imItsStereo.reserve(num_agents);
    tsIts.reserve(num_agents);

    // Initialize mono and stereo image its
    if (sensor_type == ORB_SLAM2::MONOCULAR) {
        for (size_t i = 0; i < num_agents; i++) {
            imItsMonocular[i] = vvstrImagesMonocular[i].begin();
        }
    }
    else if (sensor_type == ORB_SLAM2::STEREO) {
        for (size_t i = 0; i < num_agents; i++) {
            imItsStereo[i] = vvpstrImagesStereo[i].begin();
        }
    }
    // Initialize timestamp its
    for (size_t i = 0; i < num_agents; i++) {
        tsIts[i] = vvnTimestamps[i].begin();
    }

    size_t breakCounter = 0;

    while (true) {

        for (size_t i = 0; i < num_agents; i++) {

            if (sensor_type == ORB_SLAM2::MONOCULAR) {
                vector<string>::iterator imIt = imItsMonocular[i];
                if (imIt != vvstrImagesMonocular[i].end()) {
                    cv::Mat *im = new cv::Mat();
                    *im = cv::imread(*imIt, CV_LOAD_IMAGE_UNCHANGED);
                    vmatImagesMonocular[i] = im;
                    if ((*im).empty()) {
                        cerr << endl << "Failed to load this image: " 
                            << string(*imIt) << endl;
                        return 1;
                    }
                }
            }
            else if (sensor_type == ORB_SLAM2::STEREO) {
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

                    if (seq_type == "kitti") {
                        vpmatImagesStereo[i] = make_pair(imLeft, imRight);
                    }
                    else if (seq_type == "euroc") {
                        cv::Mat *imLeftRect = new cv::Mat();
                        cv::Mat *imRightRect = new cv::Mat();
                        cv::remap(*imLeft, *imLeftRect,
                            M1l, M2l, cv::INTER_LINEAR);
                        cv::remap(*imRight, *imRightRect,
                            M1r, M2r, cv::INTER_LINEAR);
                        vpmatImagesStereo[i] = make_pair(imLeftRect, imRightRect);
                    }
                }
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
                if (sensor_type == ORB_SLAM2::MONOCULAR) {
                    cv::Mat im = *(vmatImagesMonocular[i]);
                    double ts = vnTimeStamps[i];
                    SLAMSystems[i]->TrackMonocular(im, ts);
                }
                else if (sensor_type == ORB_SLAM2::STEREO) {
                    cv::Mat imLeft = *(vpmatImagesStereo[i].first);
                    cv::Mat imRight = *(vpmatImagesStereo[i].second);
                    double ts = vnTimeStamps[i];
                    SLAMSystems[i]->TrackStereo(imLeft, imRight, ts);
                }
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
                if (sensor_type == ORB_SLAM2::MONOCULAR)
                    imItsMonocular[i]++;
                else if (sensor_type == ORB_SLAM2::STEREO)
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

    cout << "Press any button to continue..." << endl;
    cin.get();

    // Stop all threads
    for (auto SLAMSystem : SLAMSystems)
        SLAMSystem->Shutdown();

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
        if (sensor_type == ORB_SLAM2::MONOCULAR)
            SLAMSystem->SaveKeyFrameTrajectoryTUM(SLAMSystem->mId);
        else if (sensor_type == ORB_SLAM2::STEREO && seq_type == "kitti")
            SLAMSystem->SaveTrajectoryKITTI(SLAMSystem->mId);
        else if (sensor_type == ORB_SLAM2::STEREO && seq_type == "euroc")
            SLAMSystem->SaveTrajectoryTUM(SLAMSystem->mId);
    }

    return 0;
}


void LoadImages(
    const string &data_path, const size_t &num_agents,
    const string &seq_type, const int &sensor_type,
    vector<vector<string>> &vvstrImagesMonocular,
    vector<vector<pair<string, string>>> &vvpstrImagesStereo,
    vector<vector<double>> &vvnTimestamps) {

    // prefixes
    string strPrefixLeft;
    string strPrefixRight;
    if (seq_type == "kitti") {
        strPrefixLeft = data_path + "/image_0/";
        strPrefixRight = data_path + "/image_1/";
    }
    else if (seq_type == "euroc") {
        strPrefixLeft = data_path + "/cam0/data/";
        strPrefixRight = data_path + "/cam1/data/";
    }

    // timestamps path
    string timestamps_path;
    if (seq_type == "kitti") {
        timestamps_path = data_path + "/times.txt";
    }
    else if ((seq_type == "euroc") ) {
        timestamps_path = data_path + "/cam0/data.csv";
    }

    // paths vectors
    vector<double> vnTimestamps;
    vector<string> vstrImagesMonocular;
    vector<pair<string, string>> vpstrImagesStereo;

    // complete the paths vectors
    int i = 0;
    stringstream ss;
    string timestamp;
    double t = -1;
    ifstream fTimes;
    fTimes.open(timestamps_path.c_str());
    while (!fTimes.eof()) {
        if (seq_type == "kitti") {
            getline(fTimes, timestamp);
        }
        else if (seq_type == "euroc") {
            if (!i)
                getline(fTimes, timestamp);
            getline(fTimes, timestamp, ',');
        }
        if (!timestamp.empty()) {
            // complete kitti vectors
            if (seq_type == "kitti") {
                ss << setfill('0') << setw(6) << i;

                if (sensor_type == ORB_SLAM2::MONOCULAR) {
                    vstrImagesMonocular
                        .push_back(strPrefixLeft + ss.str() + ".png");
                }
                else if (sensor_type == ORB_SLAM2::STEREO) {
                    pair<string, string> p = make_pair(
                            strPrefixLeft + ss.str() + ".png",
                            strPrefixRight + ss.str() + ".png");
                    vpstrImagesStereo.push_back(p);
                }


                ss.str(std::string());
                ss.clear();

                ss << timestamp;
                ss >> t;
                vnTimestamps.push_back(t);
            }
            // complete euroc vectors
            else if (seq_type == "euroc") {
                ss << timestamp;

                if (sensor_type == ORB_SLAM2::MONOCULAR) {
                    vstrImagesMonocular
                        .push_back(strPrefixLeft + ss.str() + ".png");
                }
                else if (sensor_type == ORB_SLAM2::STEREO) {
                    pair<string, string> p = make_pair(
                            strPrefixLeft + ss.str() + ".png",
                            strPrefixRight + ss.str() + ".png");
                    vpstrImagesStereo.push_back(p);
                }

                ss >> t;
                vnTimestamps.push_back(t/1e9);
                getline(fTimes, timestamp);
            }
        }

        ss.str(std::string());
        ss.clear();
        t = -1;

        ++i;
    }

    // for (auto str : vpstrImagesStereo) {
    //     cout << str.first << " " << str.second << endl;
    // }

    // for (auto str : vstrImagesMonocular) {
    //     cout << str << endl;
    // }

    // split the paths vectors
    size_t length = vnTimestamps.size() / num_agents;   // length of each vector
    size_t remain = vnTimestamps.size() % num_agents;   // extra elements
    size_t begin = 0, end = 0;
    for (size_t i = 0; i < num_agents; ++i) {

        if (remain > 0) {
            end += length + 1;
            remain--;
        }
        else
            end += length;

        // timestamps
        vvnTimestamps.push_back(
            vector<double>(
                vnTimestamps.begin() + begin,
                vnTimestamps.begin() + end));

        // images
        if (sensor_type == ORB_SLAM2::MONOCULAR) {
            vvstrImagesMonocular.push_back(
                vector<string>(
                    vstrImagesMonocular.begin() + begin,
                    vstrImagesMonocular.begin() + end));
        }
        else if (sensor_type == ORB_SLAM2::STEREO) {
            vvpstrImagesStereo.push_back(
                vector<pair<string, string>>(
                    vpstrImagesStereo.begin() + begin,
                    vpstrImagesStereo.begin() + end));
        }

        begin = end;
    }
}

bool ParseArgs(int argc, char **argv,
    string &seq_type, size_t &num_agents, string &voc_path,
    string &data_path, string &sett_path, int &sensor_type) {

    int opt;
    while ((opt = getopt(argc, argv, "t:n:v:d:s:")) != -1) {
        string optstr = string(optarg);
        switch (opt) {
            case 't':
                if (optstr == "mono_kitti") {
                    seq_type = "kitti";
                    sensor_type = ORB_SLAM2::MONOCULAR;
                }
                else if (optstr == "mono_euroc") {
                    seq_type = "euroc";
                    sensor_type = ORB_SLAM2::MONOCULAR;
                }
                else if (optstr == "mono_tum") {
                    seq_type = "tum";
                    sensor_type = ORB_SLAM2::MONOCULAR;
                }
                else if (optstr == "stereo_kitti") {
                    seq_type = "kitti";
                    sensor_type = ORB_SLAM2::STEREO;
                }
                else if (optstr == "stereo_euroc") {
                    seq_type = "euroc";
                    sensor_type = ORB_SLAM2::STEREO;
                }
                else if (optstr == "rgbd_tum") {
                    seq_type = "tum";
                    sensor_type = ORB_SLAM2::RGBD;
                }
                else {
                    cout << "Unmatched sequence type argument: " << optstr << endl;
                    Usage();
                    return false;
                }
                break;
            case 'n':
                try {
                    num_agents = stoi(optstr);
                }
                catch (int e) {
                    cout << "Invalid number of agents: " << optstr << endl;
                    Usage();
                    return false;
                }
                break;
            case 'v':
                voc_path = optstr;
                break;
            case 'd':
                data_path = optstr;
                break;
            case 's':
                sett_path = optstr;
                break;
            case 'h':
                Usage();
                return false;
                break;
            case '?':
                cout << "Unmatched option: " << optopt << endl;
                Usage();
                return false;
                break;
            case ':':
                cout << "Missing argument for option: " << optopt << endl;
                Usage();
                return false;
                break;
            default:
                return false;
                break;
        }
    }

    if (seq_type.empty() || num_agents < 2 || num_agents > 4 || voc_path.empty()
        || data_path.empty() || sett_path.empty()) {
        cout << "A required option is missing or invalid." << endl;
        Usage();
        return false;
    }

    return true;
}

void Usage() {
    cout <<
        "Usage: ./stereo_kitti_multiagent [options]\n" <<
        "  Note that all of the following options (besides -h) are required.\n" <<
        "  Options:\n" <<
        "   -h                 Print this help message.\n" <<
        "   -t [seq_type]      The sequence type. Valid options: mono_kitti, mono_euroc, mono_tum, stereo_kitti, stereo_euroc, and rgbd_tum\n" <<
        "   -n [num_agents]    The number of agents to use (2-4).\n" <<
        "   -v [voc_path]      The path to the ORB vocabulary.\n" <<
        "   -d [data_path]     The path to the data sequence.\n" 
        "   -s [sett_path]     The path to the data sequence's settings yaml file." << endl;
}
