#ifndef DEFINES_H
#define DEFINES_H

namespace ORB_SLAM2 {
    // Input sensor type
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

    // Set to true to pause processing at key points around map fusions.
    // Strictly for analysis. By default only compatible with the 
    // generic_multiagent driver.
    #define MF_PAUSE false
}

#endif // DEFINES_H
