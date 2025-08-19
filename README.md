# ROS 2 기반 자율주행 차량 설계 및 구현 - 시뮬레이션 환경
"ROS 2 기반 자율주행 차량 설계 및 구현" 교재에서 사용하는 주행 환경에 대한 시뮬레이션 도구를 제공하고 있습니다."

## 초기 환경설정
```
git clone https://github.com/SKKUAutoLab/ros2_autonomous_vehicle_simulation
cd ~/ros2_autonomous_vehicle_simulation
sh install.sh
source ~/.bashrc
```


## 초기 환경설정
```
cd ~/ros2_autonomous_vehicle_simulation
export AMENT_PREFIX_PATH=''
export CMAKE_PREFIX_PATH=''
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y
```


## 패키지 빌드
```
cd ~/ros2_autonomous_vehicle_simulation
source /opt/ros/humble/setup.bash
colcon build --packages-select interfaces_pkg --allow-overriding interfaces_pkg 
source install/local_setup.bash

colcon build --symlink-install --packages-select camera_perception_pkg --allow-overriding camera_perception_pkg 
source install/local_setup.bash

colcon build --symlink-install --packages-select decision_making_pkg --allow-overriding decision_making_pkg 
source install/local_setup.bash

colcon build --symlink-install --packages-select debug_pkg --allow-overriding debug_pkg 
source install/local_setup.bash

colcon build --symlink-install --packages-select simulation_pkg --allow-overriding simulation_pkg
source install/local_setup.bash

colcon build --symlink-install --packages-select lidar_perception_pkg --allow-overriding lidar_perception_pkg
source install/local_setup.bash
```


## 시뮬레이터 실행

### 장애물 없는 환경
```
cd ~/ros2_autonomous_vehicle_simulation
sudo killall -9 gazebo gzserver gzclient; ros2 launch simulation_pkg driving_sim.launch.py
```

### 장애물 및 신호등 있는 환경
```
cd ~/ros2_autonomous_vehicle_simulation
sudo killall -9 gazebo gzserver gzclient; ros2 launch simulation_pkg mission_sim.launch.py
```

### 주차 환경
```
cd ~/ros2_autonomous_vehicle_simulation
sudo killall -9 gazebo gzserver gzclient; ros2 launch simulation_pkg parking_sim.launch.py
```
