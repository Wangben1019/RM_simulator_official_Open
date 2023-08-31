# Message type

## 1. ``nav_msgs::Odometry``：对位姿，对速度的估计

### topic:

* **``/state_estimation``**

```shell
wang@wangpc:~$ rosmsg show nav_msgs/Odometry 
# 存储机器人在自由空间中的位置和速度的估计
std_msgs/Header header
  uint32 seq		# 序列号
  time stamp		# 时间戳
  string frame_id	# 坐标ID
string child_frame_id
geometry_msgs/PoseWithCovariance pose
# 表示带有协方差矩阵的位姿估计，协方差矩阵表示其不确定度，用6*6的矩阵表示协方差，对应表示绕xyz三轴的不确定度
  geometry_msgs/Pose pose
  # 位姿，即位置和姿态，用point表示位置，用四元数表示姿态
    geometry_msgs/Point position
    # 表示自由空间中的点
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
    # 用四元数表示自由空间中的旋转
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
  # 用6*6的矩阵表示协方差
geometry_msgs/TwistWithCovariance twist
# 表示带有协方差表示不确定度的速度估计
  geometry_msgs/Twist twist
  # 表示自由空间的一组速度，包括线速度和角速度
    geometry_msgs/Vector3 linear
    # 表示自由空间的三维向量，是一个结构体
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
    # 表示自由空间的三维向量，是一个结构体
      float64 x
      float64 y
      float64 z
  float64[36] covariance
  # 6*6协方差
```



## 2. ``sensor_msgs::PointCloud2``

### topic:

* **``/terrain_map``**
* **``/livox_horizon_points``**

```shell
wang@wangpc:~$ rosmsg show sensor_msgs/PointCloud2
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height	# 点云的高度，如果是无序点云，则为1
uint32 width	# 每行点云的宽度
sensor_msgs/PointField[] fields	# 每个点的成员变量，其中name为对应成员变量的名字，datetype为变量的数据类型
  uint8 INT8=1
  uint8 UINT8=2
  uint8 INT16=3
  uint8 UINT16=4
  uint8 INT32=5
  uint8 UINT32=6
  uint8 FLOAT32=7
  uint8 FLOAT64=8
  string name
  uint32 offset
  uint8 datatype
  uint32 count
bool is_bigendian
uint32 point_step 	# 每个点占用的比特数，1个字节对应8个比特数
uint32 row_step		# 每一行占用的比特数
uint8[] data		# 为序列化后的数据，直接获得不了信息，序列化是为了方便信息传输和交换，使用时需要反序列化
bool is_dense		# 是否有非法数据点，true表示没有


```



## 3. ``nav_msgs::Path``

### topic:

* **``/path``**

```shell
wang@wangpc:~$ rosmsg show nav_msgs/Path 
# 路径
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/PoseStamped[] poses
# 代表路径的三维点坐标数组
# 表示带有时间戳和参考系的位姿
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Pose pose
  # 位姿，即位置和姿态，用point表示位置，用四元数表示姿态
    geometry_msgs/Point position
    # 表示自由空间中的点
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
    #  用四元数表示自由空间中的旋转
      float64 x
      float64 y
      float64 z
      float64 w

```



## 4. ``geometry_msgs::TwistStamped``

### topic:

* **``/cmd_vel``**

```shell
wang@wangpc:~$ rosmsg show geometry_msgs/TwistStamped 
# 表示带有时间戳和参考坐标系的速度
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Twist twist
# 表示自由空间的一组速度，包括线速度和角速度
  geometry_msgs/Vector3 linear
  # 表示自由空间的三维向量，是一个结构体，内置三个类型
  # 机器人模型的线速度
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 angular
  # 表示自由空间的三维向量，是一个结构体，内置三个类型
  # 机器人模型的角速度
    float64 x
    float64 y
    float64 z

```



## 5. ``gazebo_msgs::ModelState``

### topic:

* **``/gazebo/set_model_state``：设置机器人位姿**

```shell
wang@wangpc:~$ rosmsg show gazebo_msgs/ModelState
string model_name
geometry_msgs/Pose pose
# 位姿，即位置和姿态，用point表示位置，用四元数表示姿态
  geometry_msgs/Point position
  # 表示自由空间中的点
  # 机器人模型的在Gazebo中的位姿
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
  #  用四元数表示自由空间中的旋转
  # 机器人模型的在Gazebo中的位姿
    float64 x
    float64 y
    float64 z
    float64 w
geometry_msgs/Twist twist
# 表示自由空间的一组速度，包括线速度和角速度
  geometry_msgs/Vector3 linear
  # 表示自由空间的三维向量，是一个结构体，内置三个类型
  # 机器人模型的在Gazebo中的线速度
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 angular
   # 表示自由空间的三维向量，是一个结构体，内置三个类型
   # 机器人模型的在Gazebo中的线速度
    float64 x
    float64 y
    float64 z
string reference_frame


```



## 6. ``sensor_msgs::Joy``

```shell
wang@wangpc:~$ rosmsg show sensor_msgs/Joy
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32[] axes
int32[] buttons

```



## 7. ``geometry_msgs::PointStamped

```shell
wang@wangpc:~$ rosmsg show geometry_msgs/PointStamped 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Point point
  float64 x
  float64 y
  float64 z

```



## 8. ``geometry_msgs::PolygonStamped``

```shell
wang@wangpc:~$ rosmsg show geometry_msgs/PolygonStamped 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Polygon polygon
  geometry_msgs/Point32[] points
    float32 x
    float32 y
    float32 z


```

