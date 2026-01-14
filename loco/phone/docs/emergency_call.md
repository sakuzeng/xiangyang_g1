# Emergency Call 紧急呼叫系统

## 系统概述

Emergency Call 是一个集成了机械臂控制、视觉定位、语音交互（TTS/ASR）的自动化触屏系统。主要用于在变电站等场景下，通过语音指令触发机器人点击屏幕上的特定区域（如拨打紧急电话），并播报告警信息。

该系统通过 RESTful API 对外提供服务，支持任务排队、状态查询和异常处理。

## 系统架构

系统采用**生产者-消费者**模型，Web Server 负责接收请求并入队，后台 Worker 线程负责串行执行复杂的机械臂与语音交互任务。

### 1. 软件组件架构

系统分为接口层、任务调度层、控制层和硬件抽象层。

```mermaid
graph TB
    subgraph Layer1 ["🔌 接口层"]
        API["server_emergency_call.py<br/>(FastAPI)"]
    end

    subgraph Layer2 ["⚙️ 任务调度层"]
        Worker["Task Worker"]
        Queue["Queue"]
        Store["Task Store (Dict)"]
    end

    subgraph Layer3 ["🧠 控制核心层"]
        Interface["phone_touch_interface.py<br/>(统一入口/参数自动适配)"]
        TaskCtrl["phone_touch_task.py<br/>(动作序列控制)"]
        Locator["screen_target_locator.py<br/>(视觉定位/YOLO)"]
        IKSolver["screen_to_ik.py<br/>(逆运动学解算)"
]
    end

    subgraph Layer4 ["📡 硬件与服务层"]
        LocoClient["Unitree SDK<br/>(运动控制)"]
        Camera["Realsense SDK<br/>(视觉输入)"]
        TTS["TTS Client"]
        ASR["ASR Client"]
    end

    %% 关系
    API --> Queue
    API --> Store
    Worker --> Queue
    Worker --> Store
    
    Worker -->|调用| TTS
    Worker -->|调用| ASR
    Worker -->|执行| Interface
    
    Interface --> TaskCtrl
    TaskCtrl --> IKSolver
    TaskCtrl --> LocoClient
    IKSolver --> Locator
    Locator --> Camera
```

### 2. 触碰屏幕流程

```mermaid
sequenceDiagram
    participant API as API Server
    participant Worker as Worker线程
    participant TTS as TTS服务
    participant ASR as ASR服务
    participant Interface as touch_interface
    participant Task as TouchController
    participant IK as IK解算器
    participant Vision as 视觉定位
    participant Camera as 相机
    participant Arm as 机械臂
    participant Hand as 灵巧手

    Note over API,Worker: 📥 任务接收与排队
    API->>Worker: 任务入队(speak_msg, target_index)
    
    Note over Worker,ASR: 🎙️ 语音交互确认
    Worker->>TTS: 申请独占模式
    activate TTS
    Worker->>TTS: "是否需要拨打..."
    Worker->>ASR: 录音识别(4s, VAD)
    activate ASR
    ASR-->>Worker: 识别文本
    deactivate ASR
    
    alt 用户确认
        Note over Worker,Task: 🤖 初始化控制器
        Worker->>TTS: "正在为您拨通..."
        Worker->>Interface: touch_target(target_index)
        activate Interface
        Interface->>Interface: 检测模式(FSM ID)<br/>加载参数配置
        Interface->>Task: 创建Controller实例
        activate Task
        
        Note over Task,Hand: 🔧 硬件初始化
        Task->>Arm: 初始化左臂
        Task->>Hand: 初始化左手
        Task->>Task: 加载姿态库
        
        Note over Task,Camera: 👁️ 视觉定位
        Task->>IK: solve_for_target(index)
        activate IK
        IK->>Camera: 启动相机
        activate Camera
        IK->>Vision: detect_and_locate()
        activate Vision
        Vision->>Vision: YOLO检测<br/>深度获取(同心圆)<br/>Torso Z验证<br/>中值填补
        Vision-->>IK: Torso坐标
        deactivate Vision
        deactivate Camera
        
        Note over IK: 🧮 运动学求解
        IK->>IK: 误差修正<br/>IK求解<br/>工作空间验证
        IK-->>Task: 关节角度[7]
        deactivate IK
        
        Note over Task,Hand: 🦾 执行动作序列
        Task->>Arm: 步骤1-3: 预备姿态
        Task->>Hand: 设置手势
        Task->>Arm: 步骤4: 移至目标
        Task->>Arm: 步骤5: 手腕下压(点击)
        Task->>Hand: 步骤6: 手势复位
        Task->>Arm: 步骤7: 肘部收缩
        Task->>TTS: "报事故, {speak_msg}"
        Task->>Arm: 步骤8: 安全归位
        
        Task-->>Interface: 完成
        deactivate Task
        Interface-->>Worker: 成功
        deactivate Interface
        
    else 用户未确认
        Worker->>TTS: "好的,已取消操作"
    end
    
    Note over Worker,Interface: 🧹 资源清理
    Worker->>TTS: 释放独占模式
    deactivate TTS
    Worker->>Interface: shutdown()
    Worker-->>API: 更新任务状态
```

## 核心模块详解

以下是系统中 5 个核心代码文件的功能说明：

### 1. `server_emergency_call.py` (服务入口)
- **功能**: 提供 RESTful API 接口，负责任务的接收、排队和调度。
- **核心逻辑**:
  - 维护 `task_queue` 实现生产者-消费者模型，确保任务串行执行。
  - 集成 `TTSClient` 和 `ASRClient`，实现“询问-确认”的语音交互闭环。
  - 申请 **TTS独占模式**，防止其他语音干扰紧急呼叫流程。
  - 捕获所有下层模块抛出的异常，统一进行日志记录和语音播报。

### 2. `phone_touch_interface.py` (控制接口)
- **功能**: 作为上层服务与底层控制器的中间层，提供统一的调用入口。
- **核心逻辑**:
  - **单例模式**: 维护全局唯一的 `PhoneTouchController` 实例。
  - **参数自动适配**: 自动检测机器人当前的运动模式（走跑 ID:801 / 常规），动态加载对应的参数配置：
    - **走跑模式**: `expected_torso_z` = -0.17m, 更大的 `wrist_pitch`。
    - **常规模式**: `expected_torso_z` = -0.15m。
  - **异常边界**: 捕获并封装底层异常，向上层提供清晰的错误信息。

### 3. `screen_target_locator.py` (视觉定位)
- **功能**: 负责屏幕目标的识别与三维坐标获取。
- **核心逻辑**:
  - **YOLO 检测**: 调用远程 YOLO 服务识别屏幕上的 Grid 区域。
  - **深度获取策略 (DepthHelper)**:
    1. **直接获取**: 读取中心点深度。
    2. **同心圆搜索**: 如果中心点无效，向外扩散搜索有效深度。
    3. **中值填补**: 如果上述均失败，收集周围所有有效点取中值（抗反光干扰）。
  - **Torso Z 验证**: 将相机坐标转换为 Torso 坐标，校验 Z 值是否在预期范围内（如 -0.17m ± 5cm），有效防止深度漂移导致的误操作。

### 4. `screen_to_ik.py` (IK 解算)
- **功能**: 将屏幕目标坐标转换为机械臂的关节角度。
- **核心逻辑**:
  - **集成器**: 内部实例化 `ScreenTargetLocator`，串联视觉定位与运动学计算。
  - **IK 求解**: 使用 `ikpy` 库，根据目标坐标和当前手掌姿态约束，反解出 7 个关节角度。
  - **误差修正**: 应用测量误差补偿向量（`measurement_error`）。
  - **安全校验**: 检查解算结果的位置误差（<5cm）和是否超出机械臂工作空间（Torso X/Y Range）。

### 5. `phone_touch_task.py` (动作控制)
- **功能**: 负责执行具体的机械臂动作序列。
- **核心逻辑**:
  - **完整生命周期**: 预备 -> 接近 -> 点击 -> 收缩 -> 归位。
  - **精细点击**: 到达目标位置后，通过**手腕下压 (Wrist Pitch)** 实现点击，而非整体手臂下压，提高精度和安全性。
  - **安全收缩**: 点击完成后，优先执行灵巧手复位和肘部后撤，避免回退时刮擦屏幕。
  - **智能退出**: 实现了 `_safe_emergency_exit` 机制，在发生异常或用户中断时，强制机械臂安全归位。

## 目录结构

```text
phone/
├── server_emergency_call.py    # 🚀 服务入口 (FastAPI)
├── phone_touch_interface.py    # 🔌 控制器封装接口 (自动适配参数)
├── phone_touch_task.py         # 🧠 机械臂任务控制器 (动作序列)
├── screen_to_ik.py             # 🧮 屏幕坐标到IK解算
├── screen_target_locator.py    # 👁️ 视觉定位 (YOLO + Realsense)
├── touch_exceptions.py         # ⚠️ 自定义异常类
├── start_emergency_call.sh     # 📜 启动脚本
├── stop_emergency_call.sh      # 📜 停止脚本
└── docs/                       # 📚 文档目录
    ├── emergency_call.md       #    - 系统架构文档
    └── interface/              #    - 接口文档
        └── server_emergency_call.md
```