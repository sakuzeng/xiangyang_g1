# 坐标系相关
右手笛卡尔坐标系
右手定则：食指指向 X 轴正方向；中指指向 Y 轴正方向；拇指指向 Z 轴正方向。
在旋转方向上，也遵循右手定则：大拇指指向轴的正方向，四指弯曲的方向就是旋转的正方向。

# 手臂
## left_shoulder_pitch_link
```xml
  <joint name="left_shoulder_pitch_joint" type="revolute">
    <origin xyz="0.0039563 0.10022 0.24778" rpy="0.27931 5.4949E-05 -0.00019159"/>
    <parent link="torso_link"/>
    <child link="left_shoulder_pitch_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.0892" upper="2.6704" effort="25" velocity="37"/>
  </joint>
```
- 左肩前后摆动动作
- revolut（有限的旋转关节），continuous(无限的旋转关节)
## left_shoulder_roll_joint
```xml
  <joint name="left_shoulder_roll_joint" type="revolute">
    <origin xyz="0 0.038 -0.013831" rpy="-0.27925 0 0"/>
    <parent link="left_shoulder_pitch_link"/>
    <child link="left_shoulder_roll_link"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.5882" upper="2.2515" effort="25" velocity="37"/>
  </joint>
```
- 左肩左右摆动动作
## left_shoulder_yaw_joint
```xml
  <joint name="left_shoulder_yaw_joint" type="revolute">
    <origin xyz="0 0.00624 -0.1032" rpy="0 0 0"/>
    <parent link="left_shoulder_roll_link"/>
    <child link="left_shoulder_yaw_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.618" upper="2.618" effort="25" velocity="37"/>
  </joint>
```
- 左大臂自旋动作
## left_elbow_joint
```xml
  <joint name="left_elbow_joint" type="revolute">
    <origin xyz="0.015783 0 -0.080518" rpy="0 0 0"/>
    <parent link="left_shoulder_yaw_link"/>
    <child link="left_elbow_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0472" upper="2.0944" effort="25" velocity="37"/>
  </joint>
```
- 左肘前后摆动动作
## left_wrist_roll_joint
```xml
  <joint name="left_wrist_roll_joint" type="revolute">
    <origin xyz="0.100 0.00188791 -0.010" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <parent link="left_elbow_link"/>
    <child link="left_wrist_roll_link"/>
    <limit effort="25" velocity="37" lower="-1.972222054" upper="1.972222054"/>
  </joint>
```
- 手腕旋转动作
## left_wrist_pitch_joint
```xml
  <joint name="left_wrist_pitch_joint" type="revolute">
    <origin xyz="0.038 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <parent link="left_wrist_roll_link"/>
    <child link="left_wrist_pitch_link"/>
    <limit effort="5" velocity="22" lower="-1.614429558" upper="1.614429558"/>
  </joint>
```
- 手腕前后摆动动作
## left_wrist_yaw_joint
```xml
  <joint name="left_wrist_yaw_joint" type="revolute">
    <origin xyz="0.046 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="left_wrist_pitch_link"/>
    <child link="left_wrist_yaw_link"/>
    <limit effort="5" velocity="22" lower="-1.614429558" upper="1.614429558"/>
  </joint>
```
- 手腕左右摆动动作
## left_hand_palm_joint
```xml
  <joint name="left_hand_palm_joint" type="fixed">
    <origin xyz="0.0415 0.003 0" rpy="0 0 0"/>
    <parent link="left_wrist_yaw_link"/>
    <child link="left_hand_palm_link"/>
  </joint>
```
- 手指基础定位关节

