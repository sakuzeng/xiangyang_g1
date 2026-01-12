# 坐标系相关
右手笛卡尔坐标系
右手定则：食指指向 X 轴正方向；中指指向 Y 轴正方向；拇指指向 Z 轴正方向。
在旋转方向上,也遵循右手定则:大拇指指向轴的正方向,四指弯曲的方向就是旋转的正方向。

# 手臂
## right_shoulder_pitch_link
```xml
  <joint name="right_shoulder_pitch_joint" type="revolute">
    <origin xyz="0.0039563 -0.10021 0.24778" rpy="-0.27931 5.4949E-05 0.00019159"/>
    <parent link="torso_link"/>
    <child link="right_shoulder_pitch_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.0892" upper="2.6704" effort="25" velocity="37"/>
  </joint>
```
- 右肩前后摆动动作
- revolute(有限的旋转关节),continuous(无限的旋转关节)
- 旋转轴: Y轴 (0 1 0)
- 角度范围: -177.0° ~ 153.0°
- 最大力矩: 25 N·m
- 最大速度: 37 rad/s

## right_shoulder_roll_joint
```xml
  <joint name="right_shoulder_roll_joint" type="revolute">
    <origin xyz="0 -0.038 -0.013831" rpy="0.27925 0 0"/>
    <parent link="right_shoulder_pitch_link"/>
    <child link="right_shoulder_roll_link"/>
    <axis xyz="1 0 0"/>
    <limit lower="-2.2515" upper="1.5882" effort="25" velocity="37"/>
  </joint>
```
- 右肩左右摆动动作
- 旋转轴: X轴 (1 0 0)
- 角度范围: -129.0° ~ 91.0°
- 最大力矩: 25 N·m
- 最大速度: 37 rad/s

## right_shoulder_yaw_joint
```xml
  <joint name="right_shoulder_yaw_joint" type="revolute">
    <origin xyz="0 -0.00624 -0.1032" rpy="0 0 0"/>
    <parent link="right_shoulder_roll_link"/>
    <child link="right_shoulder_yaw_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.618" upper="2.618" effort="25" velocity="37"/>
  </joint>
```
- 右大臂自旋动作
- 旋转轴: Z轴 (0 0 1)
- 角度范围: -150.0° ~ 150.0°
- 最大力矩: 25 N·m
- 最大速度: 37 rad/s

## right_elbow_joint
```xml
  <joint name="right_elbow_joint" type="revolute">
    <origin xyz="0.015783 0 -0.080518" rpy="0 0 0"/>
    <parent link="right_shoulder_yaw_link"/>
    <child link="right_elbow_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0472" upper="2.0944" effort="25" velocity="37"/>
  </joint>
```
- 右肘前后摆动动作
- 旋转轴: Y轴 (0 1 0)
- 角度范围: -60.0° ~ 120.0°
- 最大力矩: 25 N·m
- 最大速度: 37 rad/s

## right_wrist_roll_joint
```xml
  <joint name="right_wrist_roll_joint" type="revolute">
    <origin xyz="0.100 -0.00188791 -0.010" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <parent link="right_elbow_link"/>
    <child link="right_wrist_roll_link"/>
    <limit effort="25" velocity="37" lower="-1.972222054" upper="1.972222054"/>
  </joint>
```
- 手腕旋转动作
- 旋转轴: X轴 (1 0 0)
- 角度范围: -113.0° ~ 113.0°
- 最大力矩: 25 N·m
- 最大速度: 37 rad/s

## right_wrist_pitch_joint
```xml
  <joint name="right_wrist_pitch_joint" type="revolute">
    <origin xyz="0.038 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <parent link="right_wrist_roll_link"/>
    <child link="right_wrist_pitch_link"/>
    <limit effort="5" velocity="22" lower="-1.614429558" upper="1.614429558"/>
  </joint>
```
- 手腕前后摆动动作
- 旋转轴: Y轴 (0 1 0)
- 角度范围: -92.5° ~ 92.5°
- 最大力矩: 5 N·m
- 最大速度: 22 rad/s

## right_wrist_yaw_joint
```xml
  <joint name="right_wrist_yaw_joint" type="revolute">
    <origin xyz="0.046 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="right_wrist_pitch_link"/>
    <child link="right_wrist_yaw_link"/>
    <limit effort="5" velocity="22" lower="-1.614429558" upper="1.614429558"/>
  </joint>
```
- 手腕左右摆动动作
- 旋转轴: Z轴 (0 0 1)
- 角度范围: -92.5° ~ 92.5°
- 最大力矩: 5 N·m
- 最大速度: 22 rad/s

## right_hand_palm_joint
```xml
  <joint name="right_hand_palm_joint" type="fixed">
    <origin xyz="0.0415 -0.003 0" rpy="0 0 0"/>
    <parent link="right_wrist_yaw_link"/>
    <child link="right_hand_palm_link"/>
  </joint>
```
- 手指基础定位关节
- 固定关节(fixed),不可旋转

# 左右臂对比
| 关节 | 左臂Y坐标 | 右臂Y坐标 | 说明 |
|------|----------|----------|------|
| shoulder_pitch | 0.10022 | -0.10021 | Y轴对称 |
| shoulder_roll | 0.038 | -0.038 | Y轴对称 |
| shoulder_yaw | 0.00624 | -0.00624 | Y轴对称 |
| wrist_roll | 0.00188791 | -0.00188791 | Y轴对称 |
| hand_palm | 0.003 | -0.003 | Y轴对称 |

**注意**: 左右臂关节在Y轴方向上镜像对称,角度限制范围也相应对称。