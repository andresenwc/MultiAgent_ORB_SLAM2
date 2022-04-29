# Example run commands for the generic multi-agent SLAM driver.

echo "Running Generic Multi-Agent SLAM"

# ./Examples/MultiAgent/generic_multiagent -t stereo_kitti -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/kitti/data_odometry_gray/dataset/sequences/00 -s ./Examples/Stereo/KITTI00-02.yaml

./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 4 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/V101/mav0 -s ./Examples/Stereo/EuRoC.yaml