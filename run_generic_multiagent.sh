# Example run commands for the generic multi-agent SLAM driver.

echo "Running Generic Multi-Agent SLAM"

# KITTI 00-02
# ./Examples/MultiAgent/generic_multiagent -t stereo_kitti -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/kitti/data_odometry_gray/dataset/sequences/02 -s ./Examples/Stereo/KITTI00-02.yaml

# KITTI 04-12
# ./Examples/MultiAgent/generic_multiagent -t stereo_kitti -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/kitti/data_odometry_gray/dataset/sequences/09 -s ./Examples/Stereo/KITTI04-12.yaml

# EuRoC
# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/MH01/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/MH01.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/MH01_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/MH01_1.txt

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/MH02/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/MH02.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/MH02.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/MH02_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/MH02_1.txt

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/MH03/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/MH03.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/MH03.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/MH03_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/MH03_1.txt

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/MH04/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/MH04.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/MH04.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/MH04_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/MH04_1.txt

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/MH05/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/MH05.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/MH05.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/MH05_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/MH05_1.txt

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/V101/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/V101.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/V101.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/V101_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/V101_1.txt

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/V102/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/V102.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/V102.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/V102_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/V102_1.txt

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/V103/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/V103.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/V103.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/V103_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/V103_1.txt

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/V201/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/V201.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/V201.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/V201_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/V201_1.txt

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/V202/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/V202.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/V202.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/V202_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/V202_1.txt

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/V203/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/V203.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/V203.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/V203_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/V203_1.txt
