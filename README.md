# TurtleBot3 自主避障與安防視覺巡檢系統 (ROS 2)

基於 ROS 2 (`rclpy`) 開發的 TurtleBot3 Burger 自主巡檢與動態避障套件，支援 Gazebo 模擬與實體機器人無縫部署。

## 核心功能與節點說明
* **`avoid_node` (`obstacle_avoid.py`)**：融合 360 度光達五扇區感知，實現「緊急脫困 > 動態繞柱 > 自由巡航」三層平滑閃避。
* **`goal_node` (`go_to_goal_avoid.py`)**：結合 `/odom` 里程計與 `/scan` 光達，實現指定 `(X, Y)` 座標定點導航與局部避障。
* **`patrol_node` (`waypoint_patrol.py`)**：採用有限狀態機（FSM），支援多航點無限循環巡邏、定點停留視覺巡檢觸發、超時防卡關自動跳過，以及 ROS 2 動態參數調整。

## 快速啟動指令

### 1. 編譯套件
```bash
cd ~/ros2_ws
colcon build --packages-select my_tb3_avoid
source ~/ros2_ws/install/setup.bash
```

### 2. 電腦虛擬模擬一鍵啟動（支援動態參數）
```bash
ros2 launch my_tb3_avoid sim_patrol.launch.py max_speed:=0.25 inspect_duration:=5.0
```

### 3. 實體 TurtleBot3 Burger 一鍵啟動
```bash
ros2 launch my_tb3_avoid real_patrol.launch.py
```
