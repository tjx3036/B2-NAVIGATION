## bytetrack_ros

这个目录用于将 `ByteTrack` 多目标跟踪算法与当前 ROS2 / B2 导航工程集成，同时与上游源码 `AA/ByteTrack-main` 解耦：

- 放 **ROS2 封装节点**（订阅 `/front_camera/image_raw`，调用 ByteTrack 检测+跟踪，发布视觉动态障碍物轨迹）；
- 放 ByteTrack 相关的 **配置、权重路径、脚本**。

### 环境

```bash
cd /path/to/AA/ByteTrack-main
python3 setup.py develop   # 需已安装 torch
```

### 权重（必须）

从 [ByteTrack README Model zoo](https://github.com/ifzhang/ByteTrack) 下载 **bytetrack_tiny_mot17**（Google Drive 或百度网盘），保存为：

```text
AA/ByteTrack-main/pretrained/bytetrack_tiny_mot17.pth
```

（若文件名不同，启动时用参数 `ckpt:=完整路径`。）

### 运行

```bash
cd RCI_quadruped_robot_navigation-main
source install/setup.bash
# 先起 Gazebo 相机，再：
python3 bytetrack_ros/bytetrack_bridge_node.py
```

可选参数（示例）：

```bash
python3 bytetrack_ros/bytetrack_bridge_node.py --ros-args \
  -p ckpt:=/你的路径/bytetrack_tiny_mot17.pth \
  -p device:=cpu
```

### 检测图像单独 GUI（不用 RViz）

检测到的带框图像在**独立窗口**显示，不放在 RViz 里：

```bash
# 先起桥接节点（有相机+权重），再起 GUI
python3 bytetrack_ros/vision_detection_gui.py
```

- 订阅话题：`/vision_detections_image`（桥接节点发布的画好框+ID 的图像）。
- 窗口标题：`Vision Detection (ByteTrack)`，按 **q** 退出。

### 文件

- `bytetrack_bridge_node.py`：订阅 `/front_camera/image_raw`，发布 `/vision_dynamic_obstacles`、`/vision_dynamic_markers`、`/vision_detections_image`（带框图像供 GUI 显示）。
- `vision_detection_gui.py`：仅订阅 `/vision_detections_image`，在单独 OpenCV 窗口显示检测结果。

