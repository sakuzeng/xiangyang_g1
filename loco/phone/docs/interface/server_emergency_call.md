# 告警信息拨打电话接口

## 接口信息

### 1. 触发紧急呼叫 (trigger_emergency_call)

| 接口名称 | trigger_emergency_call |
| :--- | :--- |
| **接口描述** | 将紧急呼叫任务加入队列。任务将串行执行：语音播报询问 -> 语音识别确认 -> 机械臂操作。返回任务ID用于后续状态查询。 |
| **请求方法** | POST |
| **请求路径** | `http://localhost:9000/emergency_call` |
| **Content-Type** | application/json |

**请求参数**

| 参数名称 | 类型 | 是否必须 | 默认值 | 描述 |
| :--- | :--- | :--- | :--- | :--- |
| speak_msg | string | 是 | | 告警播报内容（例如："出现跳闸"），用于语音合成播报 |
| target_index | int | 是 | | 屏幕目标区域索引（0-35），对应需要拨打的电话位置 |

**响应参数**

| 参数名称 | 类型 | 描述 |
| :--- | :--- | :--- |
| task_id | string | 任务唯一标识符 (UUID)，用于查询执行结果 |
| status | string | 当前状态，入队成功固定返回 "queued" |
| message | string | 状态描述信息（包含当前队列排队情况） |
| queue_position | int | 当前任务在队列中的位置（0表示前方无任务，即将执行） |

**请求示例**
```json
{
    "speak_msg": "财庙变财庙变/110kV.倚财线幺栋幺开关跳闸（重合成功）(模拟)",
    "target_index": 30
}
```

**返回示例**
```json
{
    "task_id": "a1b2c3d4-e5f6-7890-1234-56789abcdef0",
    "status": "queued",
    "message": "任务 [a1b2c3d4...] 已加入队列，前方排队数: 0",
    "queue_position": 0
}
```

**参数错误示例 (target_index 越界)**
```json
{
    "detail": "Target index must be between 0 and 35"
}
```

---

### 2. 查询任务状态 (get_task_status)

| 接口名称 | get_task_status |
| :--- | :--- |
| **接口描述** | 根据 task_id 查询任务的实时执行状态、完成时间以及失败原因分析。 |
| **请求方法** | GET |
| **请求路径** | `http://localhost:9000/emergency_call/status/{task_id}` |

**路径参数**

| 参数名称 | 类型 | 是否必须 | 描述 |
| :--- | :--- | :--- | :--- |
| task_id | string | 是 | 任务唯一标识符 (UUID) |

**响应参数**

| 参数名称 | 类型 | 描述 |
| :--- | :--- | :--- |
| task_id | string | 任务唯一标识符 (UUID) |
| status | string | 当前状态，可能为 `"queued"`（入队）、`"processing"`（处理中）、`"completed"`（已完成）、`"failed"`（失败） |
| created_at | string | 任务创建时间（ISO 8601 格式） |
| started_at | string | 任务开始执行时间 (未开始则为 null) |
| completed_at | string | 任务结束时间 (未结束则为 null) |
| error | string | 错误简述 (仅 status 为 failed 时存在) |
| possible_causes | list[str] | 可能的失败原因列表 (仅 status 为 failed 时存在) |
| request_data | object | 原始请求数据回显 |

**返回示例 (执行成功)**
```json
{
    "task_id": "a1b2c3d4-e5f6-7890-1234-56789abcdef0",
    "status": "completed",
    "created_at": "2023-10-27T10:00:00.123456",
    "started_at": "2023-10-27T10:00:01.000000",
    "completed_at": "2023-10-27T10:00:15.000000",
    "request_data": {
        "speak_msg": "财庙变财庙变/110kV.倚财线幺栋幺开关跳闸（重合成功）(模拟)",
        "target_index": 30
    }
}
```

**返回示例 (执行失败 - 包含排查建议)**
```json
{
    "task_id": "a1b2c3d4-e5f6-7890-1234-56789abcdef0",
    "status": "failed",
    "created_at": "2023-10-27T10:00:00.123456",
    "error": "语音交互超时或未检测到语音",
    "error_type": "TouchSystemError",
    "possible_causes": [
        "麦克风未连接或静音",
        "环境噪音过大导致无法检测语音起始点",
        "UDP端口被占用"
    ],
    "request_data": {
        "speak_msg": "财庙变财庙变/110kV.倚财线幺栋幺开关跳闸（重合成功）(模拟)",
        "target_index": 30
    }
}
```

---

### 3. 服务健康检查 (health_check)

| 接口名称 | health_check |
| :--- | :--- |
| **接口描述** | 检查服务是否存活。 |
| **请求方法** | GET |
| **请求路径** | `http://localhost:9000/health` |

**返回示例**
```json
{
    "status": "ok",
    "service": "emergency_call_service"
}
```
