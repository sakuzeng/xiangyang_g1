ROBOT = "g1"  # 机器人型号
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene.xml"  # 机器人场景文件
DOMAIN_ID = 1  # DDS域ID
INTERFACE = "lo"  # 本地回环接口(推荐) - 因为仿真和测试都在同一台G1上

USE_JOYSTICK = 0  # 禁用手柄(G1开发板通常不接手柄)
JOYSTICK_TYPE = "xbox"  # 手柄类型(不使用可忽略)
JOYSTICK_DEVICE = 0  # 手柄设备编号(不使用可忽略)

PRINT_SCENE_INFORMATION = True  # 打印场景信息(用于调试)
ENABLE_ELASTIC_BAND = False  # 禁用弹性带(G1不需要)

SIMULATE_DT = 0.005  # 仿真频率 200Hz
VIEWER_DT = 0.02  # 可视化刷新 50FPS
