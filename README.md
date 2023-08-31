**Markdown之[ReadCode.md](ReadCode.md)：对各个模块代码的节点名称、订阅话题、发布话题以及作用的整理**

**Markdown之[CommunicationData.md](CommunicationData.md)：运行程序后ros的话题，服务，参数，节点等(命令得出，可读性不强)**

**Markdown之[Message_type.md](Message_type.md)：部分话题所用的消息类型（搞清楚逻辑的中间产物罢了）**

**Date：2.4：上传官方仿真代码，并添加了部分注释，主体部分未阅读，首先搞清楚各个模块的作用**

**Date：2.5：继续完善注释搞清楚各个模块的作用，统计了各个节点的订阅/发布话题，添加了运行程序后的``rqt_graph``计算图以查看节点间关系，图片存储在文件夹``rqt_graph``中，具体关系写在[ReadCode.md](ReadCode.md)中**

**Date：2.6：开始探索整体仿真逻辑，写在[ReadCode.md](ReadCode.md)中，添加了部分消息类型，写在[Message_type.md](Message_type.md)中,并对消息类型进行了部分注释，以便搞清楚各个话题的作用以及整体的联系**

**Date：2.7：探索仿真逻辑部分模块，写在[ReadCode.md](ReadCode.md)中，只写了部分话题的逻辑，缺失部分话题待以后补上**

**Date：2.10：添加功能包[loam_interface](https://github.com/jizhang-cmu/ground_based_autonomy_basic)，[livox_ros_dirver](https://github.com/Livox-SDK/livox_ros_driver)，安装了[Livox-SDK](https://github.com/Livox-SDK/Livox-SDK)，其中``livox_ros_driver``为``LIO-Livox``的前置依赖，在``LIO-Livox``中添加了对``README.md``文件的翻译版本，（后续：白翻译了，寄，翻译到下面发现不支持MID360，麻了）**



# Livox RM Simulator

This repository contains a simulation environment of a ground vehicle equipped with a [Livox-Mid360](https://www.livoxtech.com/mid360) Lidar sensor, which provide pointcloud samples with Livox pattern for navigation and obstacle avoidance.



该存储库包含一个地面车辆的仿真环境，该地面车辆配备了[Livox-Mid360](https://www.livoxtech.com/mid360)激光雷达传感器，为导航和避障提供Livox模式的点云样本。

## Prerequisites:

It is reconmmended to run this simulation in a host rather than a virtual environment (would be better with an NVIDIA GPU). if your computer is poor, the frequency of publishing simulated message will be very low.

This repository has been tested on Ubuntu 18.04 & Ubuntu 20.04 (w/ && w/o NVIDIA GPU).

- ROS (package ros-...-desktop-full)



## how to compile:

```shell
git clone <this_repository>
cd livox_rm_simulator
catkin_make -j<thread_number>
```



## Usage:

**Launch the Livox-Mid360&RM simulator:**

```shell
source devel/setup.bash
roslaunch vehicle_simulator livox_rm_simulator.launch
```



## note:

- The Livox-Mid360 Lidar is mounted flat on the top of the vehicle.

  - Livox-Mid360激光雷达平放在车辆顶部。

- The gound vehicle is initially located in the starting zone of red team, the default position can be adjusted through param 'vehicleX' and 'vehicleY' in 'src/vehicle/livox_rm.launch'.

  - gound车辆最初位于红色车队的起步区，默认位置可通过“src/vehicle/livox_rm.launch”中的参数“vehicleX”和“vehicle Y”进行调整。

- keytopic

  There are two key output topics, one is the pointcloud from Livox-Mid360 Lidar, and the other is the pose of the vehicle. The details are as follows.

  有两个关键的输出主题，一个是Livox-Mid360激光雷达的点云，另一个是车辆的姿态。详情如下。

  |    Topic name     |           Type           |       Frequence        |                       Note                       |
  | :---------------: | :----------------------: | :--------------------: | :----------------------------------------------: |
  | /registered_scan  | sensor_msgs::PointCloud2 | 10hz (20000 pts/frame) |         simulated Livox-Mid360 sampling          |
  | /state_estimation |    nav_msgs::Odometry    |         200hz          | simulated pose (ground truth data without noise) |

- The repository provides an example of adopting the Livox-Mid360 to navigate to a target position and to avoid the nearby obstacles. The algorithm is based on the [ground_based_autonomy_basic](https://www.cmu-exploration.com) from Ji Zhang, which employed a motion primitive method to calculate the collision-free path and follow to the target. The simulated vehicle follows a simple differential motion model. Click the 'Waypoint' button in RVIZ to select a target for the robot.



​	该存储库提供了一个采用Livox-Mid360导航到目标位置并避开附近障碍物的示例。该算法基于[ground_based_autonomy_basic](https://www.cmu-exploration.com)Ji Zhang，他使用运动原始方法来计算无碰撞路径并跟踪目标。模拟车辆遵循简单的差分运动模型。单击RVIZ中的“``Waypoint``点”按钮，为机器人选择目标。

## Acknowledge

1. Livox-SDK. [livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation)
2. Ji Zhang CMU. [ground_based_autonomy_basic](https://www.cmu-exploration.com)

