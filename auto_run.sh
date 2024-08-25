#!/bin/bash

.  ~/.bashrc
.  /opt/ros/galactic/setup.bash

. ~/avees_ws/install/setup.bash
# 변수 정의
period_values=(80 90)
delay_values=(10 50 100 150)

# 설정 파일 경로
config_file="/home/nvidia/avees_ws/src/carla-virtual-platoon/carla-virtual-platoon/config/sync_config.yaml"
log_dir="/home/nvidia/ros2_ws/logfiles"
dest_dir="/home/nvidia/ros2_ws/logfiles/test"


# 모든 조합에 대해 반복 실행
for period in "${period_values[@]}"; do
  for delay in "${delay_values[@]}"; do
    
    # 설정 파일 수정
    sed -i "s/period\/velocity_planner : [0-9]\+/period\/velocity_planner : $period/" $config_file
    sed -i "s/delay\/velocity_planner : [0-9]\+/delay\/velocity_planner : $delay/" $config_file
    
    # sleep 1초 후 각 터미널에서 명령 실행
    sleep 1s
    gnome-terminal --tab -- ros2 launch virtual-truck-control LV.launch.py carla_sync_with_delay:=true
    gnome-terminal --tab -- ros2 launch virtual-truck-control FV1.launch.py carla_sync_with_delay:=true
    gnome-terminal --tab -- ros2 launch virtual-truck-control FV2.launch.py carla_sync_with_delay:=true
    gnome-terminal --tab -- ros2 launch virtual-truck-control FV3.launch.py carla_sync_with_delay:=true

    # 5초 대기 후 다음 명령 실행
    sleep 5s
    gnome-terminal --tab -- ros2 run carla-sync-manager main

    # 5초 대기 후 플래툰 실행
    sleep 5s
    ros2 launch carla-virtual-platoon carla-virtual-platoon.launch.py NumTrucks:=4
    
    sleep 15s
    # 각 조합에 대해 로그 파일을 복사하고 삭제
    new_folder="$dest_dir/${period}-${delay}"
    mkdir -p "$new_folder"

    mv "$log_dir/truck1.csv" "$new_folder/"
    mv "$log_dir/truck2.csv" "$new_folder/"
    mv "$log_dir/truck3.csv" "$new_folder/"
    mv "$log_dir/truck4.csv" "$new_folder/"

    echo "********************************************************************************************"
    echo "Logs for period=${period}, delay=${delay} saved to ${new_folder} and original files deleted."
    echo "********************************************************************************************"

  done
done
