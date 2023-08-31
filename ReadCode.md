# ReadCode
### 1. planner

#### 1.1 local_planner

**局部路径规划planner,输入机器人里程计和目标路点,输出控制指令cmd_vel.用于局部路径规划导航**

**车辆局部路径规划器**

##### ``localPlanner.cpp``：节点名称：``localPlanner``

**作用：localplanner作用是规划局部路径**

* **订阅话题**

  * **话题``/state_estimation``**
    * **type：``nav_msgs::Odometry``：里程计信息**
    
  * **话题``/registered_scan``**
    * **type：``sensor_msgs::PointCloud2``**
    
  * **话题``/terrain_map``**
    * **type：``sensor_msgs::PointCloud2``**
    
  * **话题``/joy``**
  
    * **type：``sensor_msgs::Joy``**
  
  * **话题``/way_point``**
    * **type：``geometry_msgs::PointStamped``**
    
  * **话题``/speed``**
    * **type：``std_msgs::Float32``**
    
  * **话题``/navigation_boundary``**
  
    * **type：``geometry_msgs::PolygonStamped``**
  
  * **话题``/added_obstacles``**
  
    * **type：``sensor_msgs::PointCloud2``**
  
  * **话题``/check_obstacle``**
  
    * **type：``std_msgs::Bool``**
* **发布话题**
  
  * **话题``/path``**
    * **type：``nav_msgs::Path``**
  
  * **话题``/free_paths``**
    * **type：``sensor_msgs::PointCloud2``**
  
  
##### ``pathFollower.cpp``：节点名称：``pathFollower``

**作用：作用是根据规划的局部轨迹进行轨迹跟踪,核心思想与pid类似**

* **订阅话题**
  * **话题``/state_estimation``**
    * **type：``nav_msgs::Odometry``：里程计信息**
  * **话题``/path``**
    * **type：``nav_msgs::Path``**
  * **话题``/joy``**
    * **type：``sensor_msgs::Joy``**
  * **话题``/speed``**
    * **type：``std_msgs::Float32``**
  * **话题``/stop``**
    * **type：``std_msgs::Int8``**

* **发布话题**
  * **话题``/cmd_vel``**
    * **type：``geometry_msgs::TwistStamped``**


#### 1.2 terrain_analysis

**地面点云信息分割：将点云转换到车辆坐标系下，根据需要建立地面体素网格，然后将点云信息填充网格，根据车辆的特性筛选出合适的点云，求解最小的点云的店面高度，基于此，将小于车辆高度，大于附近的最小值，将这部分点云信息放到地面的点云信息中，从而将地面的点云提取出来发布出去**

**地形分析,叠加多帧点云,将机器人周围区域的地面进行可行驶区域分析,不能通行的区域会以点云的形式传输给localPlanner,用于localPlanner避障.**

##### ``terrainAnalysis.cpp``：节点名称：``terrainAnalysis``

* **订阅话题**
  * **话题``/state_estimation``**
    * **type：``nav_msgs::Odometry``：里程计信息**
    * **发布节点：``vehicleSimulator``**
    * **回调函数``odometryHandler``**
  * **话题``/registered_scan``**
    * **type：``sensor_msgs::PointCloud2``**
    * **发布节点：``vehicleSimulator``**
    * **回调函数``laserCloudHandler``**
      * **将满足一定条件的当前帧点云保存到``laserCloudCrop``中，点的强度设置为激光帧的时间差**
      * **然后进入主函数，对于``laserCloudCrop``中每一个点，获取其二维栅格坐标索引``indX``,``indY``**
  * **话题``/joy``**
    * **type：``sensor_msgs::Joy``**
    * **发布节点：**
  * **话题``/map_clearing``**
    * **type：``std_msgs::Float32``**
    * **发布节点：````**
* **发布话题**
  * **话题``/terrain_map``**
    * **type：``sensor_msgs::PointCloud2``**

#### 1.3 terrain_analysis_ext

**拓展地面点云信息分割**

##### ``vehicleSimulator.cpp``：节点名称：``terrainAnalysisExt``

* **订阅话题**
  * **话题``/state_estimation``**
    * **type：``nav_msgs::Odometry``**
  * **话题``/registered_scan``**
    * **type：``sensor_msgs::PointCloud2``**
  * **话题``/joy``**
    * **type：``sensor_msgs::Joy``**
  * **话题``/cloud_clearing``**
    * **type：``std_msgs::Float32``**
  * **话题``/terrain_map``**
    * **type：``sensor_msgs::PointCloud2``**
* **发布话题**
  * **话题``/terrain_map_ext``**
    * **type：``sensor_msgs::PointCloud2``**

### 2. laser_simulator

#### 2.1 livox_laser_simulation

**激光雷达模拟器？**

#### 2.2 sensor_scan_generation

##### ``sensorScanGeneration.cpp``：节点名称：``sensor_scan``

* **订阅话题``/state_estimation``和``/registered_scan``由节点``vehicleSimulator``发布**
  * **话题``/state_estimation``：模拟姿态。**
    * **type：``nav_msgs::Odometry``：里程计信息**
    * **发布节点：``vehicleSimulator``**

  * **话题``/registered_scan``模拟Livox-Mid360采样。**
    * **type：``sensor_msgs::PointCloud2``**
    * **发布节点：``vehicleSimulator``**

* **执行回调函数``laserCloudAndOdometryHandler``：将激光雷达信息进行了逆变换，转化到map的坐标系下，然后发布了两个话题。**
* **发布话题为``/state_estimation_at_scan``和``/sensor_scan``**
  * **话题``/state_estimation_at_scan``**
    * **type：``nav_msgs::Odometry``：里程计信息**
    * **注：等同于``/state_estimation``**
    
  * **话题``/sensor_scan``**
    * **type：``sensor_msgs::PointCloud2``**
    * **注：世界坐标系下的点云转换至传感器Lidar坐标系下**
  
* **将世界坐标系下的点云转换至传感器Lidar坐标系下**

### 3. tools

#### 3.1 visualization_tools

**可视化工具,将探索过程中的三个实验指标曲线进行可视化,通过matplotlib绘图显示出来,包括探索体积 探索路程 每次规划计算时间.**

##### ``visualizationTools.cpp``：节点名称：``visualizationTools``

* **订阅话题**
  * **话题``/state_estimation``**
    * **type：``nav_msgs::Odometry``**
  * **话题``/registered_scan``**
    * **type：``sensor_msgs::PointCloud2``**
  * **话题``/runtime``**
    * **type：``std_msgs::Float32``**
* **发布话题**
  * **话题``/overall_map``**
    * **type：``sensor_msgs::PointCloud2``**
  * **话题``/explored_areas``**
    * **type：``sensor_msgs::PointCloud2``**
  * **话题``/trajectory``**
    * **type：``sensor_msgs::PointCloud2``**
  * **话题``/explored_volume``**
    * **type：``std_msgs::Float32``**
  * **话题``/traveling_distance``**
    * **type：``std_msgs::Float32``**
  * **话题``/time_duration``**
    * **type：``std_msgs::Float32``**

#### 3.2 waypoint_rviz_plugin

**waypoint发布的rviz插件,运行此插件后,rviz就能通过waypoint按钮鼠标点击发布路点,直接进行导航.**

### 4. vehicle_simulator

**该ros包主要功能是实现了一个双轮差动底盘的模拟,接收cmd_vel速度信息,自己手写的机器人运动学微分方程来对机器人位姿进行推算,从而输出里程计odometry,而gazebo只是用来做模型显示.该包中包含了CMU制作的几个探索gazebo world,分别包含了不同的环境类型.**

##### ``vehicleSimulator.cpp``：节点名称：``vehicleSimulator``

* **订阅话题**
  * **话题``/simu_lidar_topic``**
    * **type：``sensor_msgs::PointCloud2``**
    * **注：``simu_lidar_topic``为字符串，实际为：``livox_horizon_points``，该话题由节点``/gazebo``发布**
  * **话题``/terrain_map``**
    * **type：``sensor_msgs::PointCloud2``**
  * **话题``/cmd_vel``**
    * **type：``geometry_msgs::TwistStamped``**
* **发布话题**
  * **话题``/state_estimation``**
    * **type：``nav_msgs::Odometry``**
  * **话题``/gazebo/set_model_state``**
    * **type：``gazebo_msgs::ModelState``**
    * **read：设置机器人位姿**
  * **话题``/registered_scan``**
    * **type：``sensor_msgs::PointCloud2``**

### 5. LIO-Livox

**编译时出现大规模报错时应将``CMakeLists.txt``中的``set(CMAKE_CXX_FLAGS "-std=c++11")``改为``set(CMAKE_CXX_FLAGS "-std=c++14")``**

**关于依赖[Ceres的安装](http://ceres-solver.org/installation.html)。注：最后安装的时候需要用``sudo make install``**



## 我逐渐理解一切 (ˉ(∞)ˉ)

### 关于``/vehicleSimulator``

**算了，我不理解，下班：Date：2.5**

**还不理解，下班：Date：2.6**

**``/vehicleSimulator``从``/gazebo``中接受雷达仿真到的数据，通过接收处理数据推测机器人位姿，并通过话题``/gazebo/set_model_state``与gazebo通信，设置仿真环境下机器人的位姿**



### 关于``/terrainAnalysis``

**接收来自``/vehicleSimulator``的雷达信息，话题：``/registered_scan``,该信息来源于在``/gazebo``中仿真得到的数据**

**接收来自``/vehicleSimulator``的机器人位姿信息，话题``/state_estimation``，对机器人进行位姿和速度进行估算，进行点云与车辆位置的校准，然后将点云数据填充体素网格，后经过滤波等一系列算法，将合适的点云筛选出来发布出去**



### 关于``/localPlanner``

**``localPlanner``可以接收两种点云数据，可以接受原始点云数据：话题：``/registered_scan``，也可以接收经过地面分割后的点云数据，通过回调处理后，将点云数据放入对象``plannerCloud``中，然后筛选出最优路径发布，话题为：``/path``**



### 关于``/pathFollower``

**``/pathFollower``订阅由``/localPlanner``发布的话题``/path``获取由``/localPlanner``筛选出来的最优路径，并对其进行追踪，其原理与pid的实现相似。并发送话题``/cmd_vel``控制机器人模型在仿真中改变位姿，速度等信息**



### 关于``/loamInterface``

**对于`/state_estimation`和`/registered_scan`两个话题，其在仿真中由`vehicle_simulator`发布，对于实际车辆来说，CMU设置了`loam_interface`功能包进行与`SLAM`算法的转换，从而将两个话题发布出来。**



### 

