# G1 迎宾演示 V3 (重构版) 文档

本文档详细说明了 `exec_greeting_demo.py` 及其相关组件的实现细节、使用方法和架构设计。

## 1. 简介

`exec_greeting_demo.py` 是 G1 机器人的迎宾演示程序。该程序协调了上肢动作（打招呼）、语音播报（TTS）以及底盘运动（精确移动与转向），展示了机器人接待访客的标准流程。

### 主要功能
1. **语音与动作协同**：在执行打招呼动作的同时进行语音播报。
2. **精确运动控制**：利用里程计反馈实现精确的直行和原地转向。
3. **资源管理**：
   - **TTS 独占模式**：防止演示过程中被其他语音指令打断。
   - **唤醒抑制**：演示过程中暂时屏蔽语音唤醒功能。

---

## 2. 系统架构

### 2.1 架构概览

```mermaid
graph TB
    subgraph external ["🌐 外部依赖服务"]
        direction LR
        TTS_Server["🔊 TTS HTTP 服务<br/><small>192.168.77.103:28001</small>"]
        Wake_Server["🎙️ 唤醒控制服务<br/><small>192.168.77.103:28004</small>"]
    end
    
    space1[ ]
    
    subgraph main ["🎯 主控层"]
        Main["exec_greeting_demo.py<br/><small>业务编排</small>"]
        Main_Init["初始化检查"]
        Main_Res["资源管理器"]
    end
    
    space2[ ]
    
    subgraph control ["🤖 控制层"]
        direction TB
        Greeting["GreetingSkill<br/><small>上肢动作</small>"]
        Loco["AdvancedLocomotion<br/><small>运动控制</small>"]
        TTS_Client["TTSClient<br/><small>语音客户端</small>"]
    end
    
    space3[ ]
    
    subgraph sdk ["📡 SDK层"]
        direction LR
        Arm_SDK["Arm SDK<br/><small>手臂控制</small>"]
        Hand_SDK["Hand SDK<br/><small>灵巧手</small>"]
        Sport_SDK["Move SDK<br/><small>移动控制</small>"]
        Odom["Odometry<br/><small>里程计</small>"]
    end
    
    space4[ ]
    
    subgraph hardware ["⚙️ 硬件层"]
        direction LR
        H1["右臂关节"]
        H2["右手电机"]
        H3["运动控制"]
        H4["IMU传感器"]
    end
    
    Main --> Main_Init
    Main --> Main_Res
    Main_Res -.->|独占控制| TTS_Client
    Main_Res -.->|暂停/恢复| Wake_Server
    
    Main --> Greeting
    Main --> Loco
    Main --> TTS_Client
    
    Greeting --> Arm_SDK
    Greeting --> Hand_SDK
    Greeting -.->|异步触发| TTS_Client
    
    Loco --> Sport_SDK
    Loco --> Odom
    
    TTS_Client -->|HTTP POST| TTS_Server
    
    Arm_SDK --> H1
    Hand_SDK --> H2
    Sport_SDK --> H3
    Odom --> H4
    
    space1 ~~~ main
    space2 ~~~ control
    space3 ~~~ sdk
    space4 ~~~ hardware
    
    style Main fill:#FFE6E6,stroke:#FF6666,stroke-width:3px
    style Main_Res fill:#FFF4E6,stroke:#FFAA33
    style Greeting fill:#E6F3FF,stroke:#3399FF,stroke-width:2px
    style Loco fill:#E6F3FF,stroke:#3399FF,stroke-width:2px
    style TTS_Client fill:#FFE6F3,stroke:#FF66CC
    style TTS_Server fill:#D4EDDA,stroke:#28A745,stroke-width:2px
    style Wake_Server fill:#D4EDDA,stroke:#28A745
    style Odom fill:#FFF9E6,stroke:#FFB366,stroke-dasharray: 5 5
    
    style space1 fill:none,stroke:none
    style space2 fill:none,stroke:none
    style space3 fill:none,stroke:none
    style space4 fill:none,stroke:none
    
    style external fill:#F0FFF0
    style main fill:#FFFEF0
    style control fill:#FFF8F0
    style sdk fill:#F0F8FF
    style hardware fill:#F5F5F5
```

### 2.2 核心模块依赖

演示程序依赖以下核心模块：

| 模块 | 文件路径 | 功能描述 |
| :--- | :--- | :--- |
| **主程序** | `exec_greeting_demo.py` | 业务逻辑编排，负责初始化和按步骤执行演示流程。 |
| **迎宾技能** | `.../skills/greeting_skill.py` | 封装了手臂和灵巧手的控制逻辑，加载预设姿态文件，执行"挥手"动作序列。 |
| **高级运动** | `.../common/advanced_locomotion.py` | 基于里程计（Odometry）闭环控制底盘，提供 `move_forward_precise` 和 `turn_angle` 方法。 |
| **TTS 客户端** | `.../common/tts_client.py` | 处理 HTTP TTS 请求，管理独占模式（Exclusive Mode）。 |

---

## 3. 完整业务流程

```mermaid
graph TB
    Start([🚀 程序启动]) --> Init[初始化 SDK & 模块]
    
    Init --> CheckInit{初始化成功?}
    CheckInit -- ❌ 失败 --> End([❌ 程序退出])
    
    CheckInit -- ✅ 成功 --> ReqTTS[申请 TTS 独占权]
    ReqTTS --> CheckTTS{获取独占权?}
    CheckTTS -- ❌ 失败<br/><small>等待3秒超时</small> --> End
    
    CheckTTS -- ✅ 成功 --> WakeLock[暂停唤醒功能]
    
    WakeLock --> Step1[步骤1️⃣: 打招呼动作 + 语音播报]
    Step1 --> Step2[步骤2️⃣: 左转 90°]
    Step2 --> Step3[步骤3️⃣: 前进 0.9米]
    Step3 --> Step4[步骤4️⃣: 右转 90°]
    
    Step4 --> Cleanup[🧹 清理资源]
    
    Cleanup --> ReleaseTTS[释放 TTS 独占]
    ReleaseTTS --> WakeUnlock[恢复唤醒功能]
    WakeUnlock --> End
    
    style Start fill:#D4EDDA,stroke:#28A745,stroke-width:2px
    style End fill:#F8D7DA,stroke:#DC3545,stroke-width:2px
    style CheckInit fill:#FFF4E6,stroke:#FFAA33
    style CheckTTS fill:#FFF4E6,stroke:#FFAA33
    style ReqTTS fill:#FFE6E6,stroke:#FF6666
    style WakeLock fill:#E6F3FF,stroke:#3399FF
    style Step1 fill:#E6FFE6,stroke:#66CC66,stroke-width:2px
    style Step2 fill:#F3E6FF,stroke:#9966FF
    style Step3 fill:#F3E6FF,stroke:#9966FF
    style Step4 fill:#F3E6FF,stroke:#9966FF
    style Cleanup fill:#F0F0F0,stroke:#999999
```

### 3.1 步骤1详解：迎宾动作序列

```mermaid
sequenceDiagram
    participant Main as 主程序
    participant Greeting as GreetingSkill
    participant Arm as 手臂SDK
    participant Hand as 灵巧手SDK
    participant TTS as TTS Client
    participant Speaker as TTS服务器
    
    Note over Main,Speaker: 🎬 步骤1: 打招呼
    
    Main->>Greeting: perform_hello_sequence()
    activate Greeting
    
    Note over Greeting,Hand: 🤚 阶段1: 举手准备
    Greeting->>Arm: 移动到 hello1 姿态
    activate Arm
    Arm->>Arm: 关节插值运动
    Arm-->>Greeting: 到达目标位置
    deactivate Arm
    
    Note over Greeting,Hand: ✋ 阶段2: 张开手掌
    Greeting->>Hand: 切换到 hello 姿态
    activate Hand
    Hand->>Hand: 五指展开
    Hand-->>Greeting: 手势完成
    deactivate Hand
    
    Note over Greeting,Speaker: 🔊 阶段3: 触发语音 (异步)
    Greeting->>TTS: speak(text, wait=False)
    activate TTS
    TTS->>Speaker: POST /speak_msg<br/><small>source=greeting_demo</small>
    activate Speaker
    Speaker-->>TTS: 任务已接收
    TTS-->>Greeting: 立即返回
    deactivate TTS
    
    Note over Greeting,Hand: 👋 阶段4: 挥手动作
    loop 挥手3次
        Greeting->>Arm: hello2 ↔ hello3
        Arm->>Arm: 往复摆动
    end
    
    par 并行执行
        Speaker->>Speaker: 文本转语音
        Speaker->>Speaker: 音频播放
    end
    deactivate Speaker
    
    Note over Greeting,Hand: 🤏 阶段5: 闭合手掌
    Greeting->>Hand: 切换到 close 姿态
    activate Hand
    Hand->>Hand: 五指收拢
    Hand-->>Greeting: 完成
    deactivate Hand
    
    Note over Greeting,Hand: 🙌 阶段6: 放下手臂
    Greeting->>Arm: 返回 nature 姿态
    activate Arm
    Arm->>Arm: 回归自然站姿
    Arm-->>Greeting: 完成
    deactivate Arm
    
    Greeting-->>Main: ✅ 动作序列完成
    deactivate Greeting
    
    Note over Main,Speaker: 整个过程约 8-10 秒
```

### 3.2 步骤2-4详解：底盘精确运动

```mermaid
sequenceDiagram
    participant Main as 主程序
    participant Loco as AdvancedLocomotion
    participant Sport as Move SDK
    participant Odom as 里程计
    
    Note over Main,Odom: 🔄 步骤2: 左转 90°
    Main->>Loco: turn_angle(-90°)
    activate Loco
    
    Loco->>Odom: 获取初始 yaw 角
    Odom-->>Loco: yaw_start
    
    loop 闭环控制
        Loco->>Sport: 发送角速度指令<br/><small>0.50 rad/s</small>
        Sport->>Sport: 执行旋转
        Loco->>Odom: 读取当前 yaw
        Odom-->>Loco: yaw_current
        Loco->>Loco: 计算累积旋转角<br/>Δyaw = yaw_current - yaw_start
        
        alt 接近目标 (剩余 < 20°)
            Loco->>Loco: 线性减速<br/>v = v_max * (剩余角/20°)
        end
        
        alt 到达目标 (误差 < 3°)
            Loco->>Sport: 停止运动
            Note over Loco: ✅ 转向完成
        else 超过目标 1.2 倍
            Loco->>Sport: 强制停止
            Note over Loco: ⚠️ 过转保护触发
        end
    end
    
    Loco-->>Main: 完成
    deactivate Loco
    
    Note over Main,Odom: 🚶 步骤3: 前进 0.9米
    Main->>Loco: move_forward_precise(0.9)
    activate Loco
    
    Loco->>Odom: 获取初始位置 (x0, y0)
    Odom-->>Loco: 起点坐标
    
    loop 闭环控制
        Loco->>Sport: 发送线速度指令<br/><small>0.30 m/s</small>
        Sport->>Sport: 执行前进
        Loco->>Odom: 读取当前位置 (x, y)
        Odom-->>Loco: 当前坐标
        Loco->>Loco: 计算移动距离<br/>d = √[(x-x0)² + (y-y0)²]
        
        alt 接近目标 (剩余 < 0.2m)
            Loco->>Loco: 线性减速<br/>v = v_max * (剩余/0.2)
        end
        
        alt 到达目标 (误差 < 0.05m)
            Loco->>Sport: 停止运动
            Note over Loco: ✅ 移动完成
        end
    end
    
    Loco-->>Main: 完成
    deactivate Loco
    
    Note over Main,Odom: 🔄 步骤4: 右转 90°
    Main->>Loco: turn_angle(+90°)
    Note over Loco,Odom: 重复步骤2流程 (方向相反)
    Loco-->>Main: 完成
```

---