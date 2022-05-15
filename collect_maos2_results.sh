# Collects various results for Multi-Agent ORB-SLAM2 using the generic driver.

# KITTI
# ./Examples/MultiAgent/generic_multiagent -t stereo_kitti -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/kitti/data_odometry_gray/dataset/sequences/00 -s ./Examples/Stereo/KITTI00-02.yaml
# cat SLAM0 SLAM1 > Analysis/kitti/traj_exp/trial4/00.txt
# mv SLAM0 Analysis/kitti/traj_exp/trial4/00_0.txt
# mv SLAM1 Analysis/kitti/traj_exp/trial4/00_1.txt
# mv stats.csv Analysis/kitti/traj_exp/trial4/00_stats.csv

# ./Examples/MultiAgent/generic_multiagent -t stereo_kitti -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/kitti/data_odometry_gray/dataset/sequences/02 -s ./Examples/Stereo/KITTI00-02.yaml
# cat SLAM0 SLAM1 > Analysis/kitti/traj_exp/trial4/02.txt
# mv SLAM0 Analysis/kitti/traj_exp/trial4/02_0.txt
# mv SLAM1 Analysis/kitti/traj_exp/trial4/02_1.txt
# mv stats.csv Analysis/kitti/traj_exp/trial4/02_stats.csv

# ./Examples/MultiAgent/generic_multiagent -t stereo_kitti -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/kitti/data_odometry_gray/dataset/sequences/05 -s ./Examples/Stereo/KITTI04-12.yaml
# cat SLAM0 SLAM1 > Analysis/kitti/traj_exp/trial4/05.txt
# mv SLAM0 Analysis/kitti/traj_exp/trial4/05_0.txt
# mv SLAM1 Analysis/kitti/traj_exp/trial4/05_1.txt
# mv stats.csv Analysis/kitti/traj_exp/trial4/05_stats.csv

# ./Examples/MultiAgent/generic_multiagent -t stereo_kitti -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/kitti/data_odometry_gray/dataset/sequences/06 -s ./Examples/Stereo/KITTI04-12.yaml
# cat SLAM0 SLAM1 > Analysis/kitti/traj_exp/trial4/06.txt
# mv SLAM0 Analysis/kitti/traj_exp/trial4/06_0.txt
# mv SLAM1 Analysis/kitti/traj_exp/trial4/06_1.txt
# mv stats.csv Analysis/kitti/traj_exp/trial4/06_stats.csv

# ./Examples/MultiAgent/generic_multiagent -t stereo_kitti -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/kitti/data_odometry_gray/dataset/sequences/07 -s ./Examples/Stereo/KITTI04-12.yaml
# cat SLAM0 SLAM1 > Analysis/kitti/traj_exp/trial4/07.txt
# mv SLAM0 Analysis/kitti/traj_exp/trial4/07_0.txt
# mv SLAM1 Analysis/kitti/traj_exp/trial4/07_1.txt
# mv stats.csv Analysis/kitti/traj_exp/trial4/07_stats.csv

# EuRoC
# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/MH01/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/trial4/MH01.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/trial4/MH01_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/trial4/MH01_1.txt
# mv stats.csv Analysis/EuRoC/traj_exp/trial4/MH01_stats.csv

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/MH02/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/MH02.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/trial4/MH02.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/trial4/MH02_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/trial4/MH02_1.txt
# mv stats.csv Analysis/EuRoC/traj_exp/trial4/MH02_stats.csv

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/MH03/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/MH03.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/trial4/MH03.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/trial4/MH03_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/trial4/MH03_1.txt
# mv stats.csv Analysis/EuRoC/traj_exp/trial4/MH03_stats.csv

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/MH04/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/MH04.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/trial4/MH04.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/trial4/MH04_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/trial4/MH04_1.txt
# mv stats.csv Analysis/EuRoC/traj_exp/trial4/MH04_stats.csv

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/MH05/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/MH05.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/trial4/MH05.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/trial4/MH05_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/trial4/MH05_1.txt
# mv stats.csv Analysis/EuRoC/traj_exp/trial4/MH05_stats.csv

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/V101/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/V101.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/trial4/V101.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/trial4/V101_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/trial4/V101_1.txt
# mv stats.csv Analysis/EuRoC/traj_exp/trial4/V101_stats.csv

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/V102/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/V102.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/trial4/V102.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/trial4/V102_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/trial4/V102_1.txt
# mv stats.csv Analysis/EuRoC/traj_exp/trial4/V102_stats.csv

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/V103/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/V103.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/trial4/V103.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/trial4/V103_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/trial4/V103_1.txt
# mv stats.csv Analysis/EuRoC/traj_exp/trial4/V103_stats.csv

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/V201/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/V201.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/trial4/V201.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/trial4/V201_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/trial4/V201_1.txt
# mv stats.csv Analysis/EuRoC/traj_exp/trial4/V201_stats.csv

# ./Examples/MultiAgent/generic_multiagent -t stereo_euroc -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/EuRoC/V202/mav0 -s ./Examples/Stereo/EuRoC.yaml -c ./Examples/Stereo/EuRoC_TimeStamps/V202.txt
# cat SLAM0 SLAM1 > Analysis/EuRoC/traj_exp/trial4/V202.txt
# mv SLAM0 Analysis/EuRoC/traj_exp/trial4/V202_0.txt
# mv SLAM1 Analysis/EuRoC/traj_exp/trial4/V202_1.txt
# mv stats.csv Analysis/EuRoC/traj_exp/trial4/V202_stats.csv

# RGBD TUM
# ./Examples/MultiAgent/generic_multiagent -t rgbd_tum -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/rgbd_tum/fr1_desk -s ./Examples/RGB-D/TUM1.yaml -a ./Examples/RGB-D/associations/fr1_desk.txt
# cat CT_SLAM0 CT_SLAM1 > Analysis/rgbd_tum/traj_exp/trial4/fr1_desk.txt
# mv CT_SLAM0 Analysis/rgbd_tum/traj_exp/trial4/fr1_desk_0.txt
# mv CT_SLAM1 Analysis/rgbd_tum/traj_exp/trial4/fr1_desk_1.txt
# mv stats.csv Analysis/rgbd_tum/traj_exp/trial4/fr1_desk_stats.csv

# ./Examples/MultiAgent/generic_multiagent -t rgbd_tum -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/rgbd_tum/fr1_desk2 -s ./Examples/RGB-D/TUM1.yaml -a ./Examples/RGB-D/associations/fr1_desk2.txt
# cat CT_SLAM0 CT_SLAM1 > Analysis/rgbd_tum/traj_exp/trial4/fr1_desk2.txt
# mv CT_SLAM0 Analysis/rgbd_tum/traj_exp/trial4/fr1_desk2_0.txt
# mv CT_SLAM1 Analysis/rgbd_tum/traj_exp/trial4/fr1_desk2_1.txt
# mv stats.csv Analysis/rgbd_tum/traj_exp/trial4/fr1_desk2_stats.csv

# ./Examples/MultiAgent/generic_multiagent -t rgbd_tum -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/rgbd_tum/fr1_room -s ./Examples/RGB-D/TUM1.yaml -a ./Examples/RGB-D/associations/fr1_room.txt
# cat CT_SLAM0 CT_SLAM1 > Analysis/rgbd_tum/traj_exp/trial4/fr1_room.txt
# mv CT_SLAM0 Analysis/rgbd_tum/traj_exp/trial4/fr1_room_0.txt
# mv CT_SLAM1 Analysis/rgbd_tum/traj_exp/trial4/fr1_room_1.txt
# mv stats.csv Analysis/rgbd_tum/traj_exp/trial4/fr1_room_stats.csv

# ./Examples/MultiAgent/generic_multiagent -t rgbd_tum -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/rgbd_tum/fr2_xyz -s ./Examples/RGB-D/TUM2.yaml -a ./Examples/RGB-D/associations/fr2_xyz.txt
# cat CT_SLAM0 CT_SLAM1 > Analysis/rgbd_tum/traj_exp/trial4/fr2_xyz.txt
# mv CT_SLAM0 Analysis/rgbd_tum/traj_exp/trial4/fr2_xyz_0.txt
# mv CT_SLAM1 Analysis/rgbd_tum/traj_exp/trial4/fr2_xyz_1.txt
# mv stats.csv Analysis/rgbd_tum/traj_exp/trial4/fr2_xyz_stats.csv

# ./Examples/MultiAgent/generic_multiagent -t rgbd_tum -n 2 -v Vocabulary/ORBvoc.txt -d ./Examples/DataSets/rgbd_tum/fr3_office -s ./Examples/RGB-D/TUM3.yaml -a ./Examples/RGB-D/associations/fr3_office.txt
# cat CT_SLAM0 CT_SLAM1 > Analysis/rgbd_tum/traj_exp/trial4/fr3_office.txt
# mv CT_SLAM0 Analysis/rgbd_tum/traj_exp/trial4/fr3_office_0.txt
# mv CT_SLAM1 Analysis/rgbd_tum/traj_exp/trial4/fr3_office_1.txt
# mv stats.csv Analysis/rgbd_tum/traj_exp/trial4/fr3_office_stats.csv
