# inno_lidar_ros
激光雷达ros驱动，现只支持IFW192S类型雷达
inno_lidar_ros驱动使用说明书

1编译与安装
inno_lidar_ros 为苏州智驰领驭在 Ubuntu 环境下的雷达ROS驱动软件包，包括了雷达驱动内核，ROS1/ROS2拓展功能。对于没有二次开发需求的用户，或是想直接使用 ROS1/ROS2 进行二次开发的用户，可直接使用本软件包，配合 ROS 自带的 RVIZ 可视化工具即可查看点云。
下载路径：https://github.com/innolidar/inno_lidar_ros.git 

1.1依赖库与安装
            1.1.1 ROS环境
若需在 ROS 环境下使用雷达驱动，则需安装 ROS1/ROS2 相关依赖库： 
ROS1:
Ubuntu 16.04 - ROS kinetic desktop-full 
Ubuntu 18.04 - ROS melodic desktop-full 
Ubuntu 20.04 - ROS noetic desktop-full 
ROS2:
Ubuntu 20.04 - ROS2 foxy desktop  
Ubuntu 22.04 - ROS2 humble desktop
安装方式： 参考 http://wiki.ros.org 
如果ROS1安装了 ROS kinetic desktop-full 版或 ROS melodic desktop-full 版，那么兼容版本其 他依赖库也应该同时被安装了，所以不需要重新安装它们以避免多个版本冲突引起的问题, 因此，强烈建议安装 desktop-full 版，这将节省大量的时间来逐个安装和配置库。
注：由于inno_lidar_ros支持播放pcap文件，那么inno_lidar_ros编译也依赖libpcap库

            1.1.2 编译与运行
inno_lidar_ros支持三种编译方式；
编译前需要的准备工作；
在inno_lidar_ros包中，找到config/config.yaml文件，依据需求配置文件参数。
配置文件中参数说明：
msg_source：0：无数据来源；1：数据来源于雷达；2：数据来源pcap文件
send_point_cloud_ros：true表示发布点云（ros1 or ros2）
Lidar_type：表示雷达类型，目前只支持IFW192S雷达
point_type：目前未使用
cloud_port：点云UDP数据端口
check_local_port: 加载矫正参数的本地端口
check_lidar_ip：加载矫正参数的设备IP
check_lidar_port：加载矫正参数的设备端口
h_start_angle：点云显示水平角最小值
h_end_angle：点云显示水平角最大值

v_start_angle：点云显示垂直角最小值
v_end_angle：点云显示垂直角最大值
min_distance：点云呈现最小距离
max_distance：点云显示最大距离
pcap_repeat：true表示本地数据循环播放
is_device_load_calibration：true表示从雷达加载矫正参数（当直连雷达时,非播放本地数据）
calibrate_folder: 表示从本地加载矫正文件的文件夹路径
pcap_file: 表示播放的本地文件
use_status: true 表示使用点云旋转平移功能
roll: 表示欧拉角roll 角（弧度值）
pitch: 表示欧拉角pitch 角（弧度值）
yaw：表示欧拉角yaw 角（弧度值）
x：表示平移向量x
y：表示平移向量y
z：表示平移向量z
ros_frame_id：帧ID
ros_send_point_cloud_topic：点云发布的topic
ros_send_imu_topic：imu发布的topic

ROS1编译：
ROS1依赖于 make 编译 (不包括 ROS2)
1. 打开工程内的 CMakeLists.txt 文件，将文件顶部的 set(COMPILE_METHOD ORIGINAL) 
改为 set(COMPILE_METHOD ORIGINAL)。 
#======================================= 
# Compile setup (ORIGINAL,CATKIN,COLCON) 
#=======================================
set(COMPILE_METHOD ORIGINAL) 
2. 返回工作空间目录，执行以下命令即可编译&运行
$cd inno_lidar_ros 
$mkdir build && cd build 
$cmake .. && make -j4 
$./ inno_lidar_node 

ROS1依赖于 ROS-catkin 编译 
1. 打开工程内的 CMakeLists.txt 文件，将文件顶部的 set(COMPILE_METHOD ORIGINAL) 
改为 set(COMPILE_METHOD CATKIN)。 
#======================================= 
# Compile setup (ORIGINAL,CATKIN,COLCON) 
#=======================================
set(COMPILE_METHOD CATKIN) 
    2. 将 inno_lidar_ros 工程目录下的 package_ros1.xml 文件重命名为 package.xml。
3. 新建一个文件夹作为工作空间，然后再新建一个名为 src 的文件夹, 将 inno_lidar_ros 工程，放入 src 文件夹内。 
4. 返回工作空间目录，执行以下命令即可编译&运行(若使用.zsh,将第二句指令替换为 
source devel/setup.zsh)。 
$catkin_make 
$source devel/setup.bash 
$roslaunch inno_lidar_ros ros1_start.launch

ROS2编译：
ROS2只能依赖于ROS2 colcon编译
1. 打开工程内的 CMakeLists.txt 文件，将文件顶部的 set(COMPILE_METHOD ORIGINAL) 
改为 set(COMPILE_METHOD COLCON)。 
#======================================= 
# Compile setup (ORIGINAL,CATKIN,COLCON) 
#=======================================
set(COMPILE_METHOD COLCON) 
    3. 将 inno_lidar_ros 工程目录下的 package_ros2.xml 文件重命名为 package.xml。
3. 新建一个文件夹作为工作空间，然后再新建一个名为 src 的文件夹, 将 inno_lidar_ros 工程，放入 src 文件夹内。 
4. 返回工作空间目录，执行以下命令即可编译&运行(若使用.zsh,将第二句指令替换为 
source install/setup.zsh)。 
$colcon build 
$source install/setup.bash 
$ros2 launch inno_lidar_ros ros2_start.launch.py



