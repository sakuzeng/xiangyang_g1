import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### ROS2 用户配置参数开始 ###################
# 数据传输格式配置
# 0 - PointCloud2 格式 (PointXYZRTL 标准点云格式)
# 1 - Livox 自定义点云格式 (包含更多雷达特定信息)
xfer_format   = 1

# 多话题模式配置
# 0 - 所有 LiDAR 共享同一个话题 (适用于单雷达或融合后发布)
#     点云话题: /livox/lidar
#     IMU话题:  /livox/imu
# 1 - 每个 LiDAR 一个独立话题 (适用于多雷达系统)
#     点云话题: /livox/lidar_192_168_123_120
#     IMU话题:  /livox/imu_192_168_123_120
multi_topic   = 0

# 数据源配置
# 0 - 从雷达实时获取数据
# 其他值 - 无效数据源
data_src      = 0

# 发布频率配置 (Hz)
# 可选值: 5.0, 10.0, 20.0, 50.0 等
# 注意: 频率越高,系统负载越大
# 限制范围: 0.5 ~ 100.0 Hz (参见 livox_ros_driver2.cpp 第 76-82 行)
publish_freq  = 20.0

# 输出数据类型
# 0 - 输出到 ROS
output_type   = 0

# 点云数据的坐标系 ID
# 所有点云数据将相对于此坐标系发布
frame_id      = 'livox_frame'

# LVX 文件路径 (用于回放录制的数据)
# 仅当 data_src 设置为从文件读取时使用
lvx_file_path = '/home/livox/livox_test.lvx'

# 命令行指定的设备序列号
# 用于识别特定的 Livox 雷达设备
cmdline_bd_code = 'livox0000000001'

# 获取当前脚本所在目录的绝对路径
cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'

# 配置文件目录路径
cur_config_path = cur_path + '../config'

# MID360 雷达的配置文件完整路径
# 该文件包含网络配置、外参等重要参数
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### ROS2 用户配置参数结束 #####################

# Livox ROS2 驱动的参数列表
# 这些参数将传递给驱动节点
livox_ros2_params = [
    {"xfer_format": xfer_format},              # 点云传输格式
    {"multi_topic": multi_topic},              # 多话题模式
    {"data_src": data_src},                    # 数据源
    {"publish_freq": publish_freq},            # 发布频率
    {"output_data_type": output_type},         # 输出数据类型
    {"frame_id": frame_id},                    # 坐标系ID
    {"lvx_file_path": lvx_file_path},          # LVX文件路径
    {"user_config_path": user_config_path},    # 用户配置文件路径
    {"cmdline_input_bd_code": cmdline_bd_code} # 设备序列号
]


def generate_launch_description():
    """
    生成 ROS2 启动描述
    此函数定义了要启动的所有节点及其配置
    """
    
    # ============ Livox 雷达驱动节点 ============
    # 负责与 MID360 雷达通信,接收并发布点云和 IMU 数据
    livox_driver = Node(
        package='livox_ros_driver2',              # ROS2 包名
        executable='livox_ros_driver2_node',      # 可执行文件名
        name='livox_lidar_publisher',             # 节点名称
        output='screen',                          # 输出到终端屏幕
        parameters=livox_ros2_params              # 传入参数列表
        # 
        # 发布的话题 (multi_topic=0 时):
        # - /livox/lidar : 点云数据 (livox_ros_driver2/CustomMsg 或 sensor_msgs/PointCloud2)
        # - /livox/imu   : IMU 数据 (sensor_msgs/Imu)
        #
        # 注意: 不需要使用 remappings，驱动默认发布到 /livox/* 话题
    )

    # ============ IMU 到 LiDAR 的静态坐标变换发布节点 ============
    # 发布 IMU 坐标系相对于 LiDAR 坐标系的固定变换关系
    # 
    # 变换参数说明:
    # - 平移向量 (米): x=0.011, y=0.02329, z=-0.04412
    #   表示 IMU 相对于 LiDAR 原点的偏移量
    # - 旋转四元数: qx=0, qy=0, qz=0, qw=1
    #   表示无旋转(IMU 和 LiDAR 坐标系方向一致)
    # - 父坐标系: livox_frame (LiDAR 坐标系)
    # - 子坐标系: livox_imu (IMU 坐标系)
    imu_to_lidar_tf = Node(
        package='tf2_ros',                        # TF2 变换库
        executable='static_transform_publisher',  # 静态变换发布器
        name='imu_to_lidar_tf',                   # 节点名称
        arguments=[
            '0.011', '0.02329', '-0.04412',       # 平移 x, y, z (米)
            '0', '0', '0', '1',                   # 四元数 qx, qy, qz, qw (无旋转)
            'livox_frame',                        # 父坐标系 (雷达坐标系)
            'livox_imu',                          # 子坐标系 (IMU坐标系)
        ],
        output='screen'                           # 输出到终端屏幕
    )

    # ============ 返回启动描述 ============
    # 按顺序启动所有定义的节点
    return LaunchDescription([
        livox_driver,        # 启动雷达驱动节点
        imu_to_lidar_tf,     # 启动静态TF发布节点
    ])