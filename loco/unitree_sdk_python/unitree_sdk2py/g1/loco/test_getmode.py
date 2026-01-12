#!/usr/bin/env python3
"""
G1机器人状态监控脚本

使用 GetFsmId 函数实时监控机器人的 FSM (有限状态机) 状态
"""

import time
import json
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


def parse_fsm_data(data):
    """解析 FSM 数据"""
    if isinstance(data, str):
        try:
            data = json.loads(data)
        except:
            return data
    
    if isinstance(data, dict) and "data" in data:
        return data["data"]
    
    return data


def get_fsm_mode_name(fsm_id):
    """将 FSM ID 转换为可读的模式名称"""
    fsm_modes = {
        0: "零力矩模式 (ZeroTorque)",
        1: "阻尼模式 (Damp)",
        3: "坐 (Sit)",
        4: "站立模式 (StandUp)",
        200: "主运控模式 (Start)",
        702: "躺下到站立 (Lie2StandUp)",
        706: "蹲下/站立切换 (Squat2StandUp)"
    }
    return fsm_modes.get(fsm_id, f"未知模式 ({fsm_id})")


def monitor_robot_status(iface="eth0", duration=30):
    """
    监控机器人状态
    
    Args:
        iface: 网络接口名称
        duration: 监控持续时间（秒）
    """
    print(f"🚀 初始化机器人连接 (接口: {iface})...")
    
    # 初始化DDS通信
    try:
        ChannelFactoryInitialize(0, iface)
        print("✓ DDS通信初始化成功")
    except Exception as e:
        print(f"❌ DDS通信初始化失败: {e}")
        return
    
    # 创建运动控制客户端
    sport_client = LocoClient()
    sport_client.SetTimeout(5.0)
    
    try:
        sport_client.Init()
        print("✓ 运动控制客户端初始化成功")
    except Exception as e:
        print(f"❌ 运动控制客户端初始化失败: {e}")
        return
    
    print(f"\n📊 开始监控机器人状态 (持续 {duration} 秒)...")
    print("=" * 80)
    print(f"{'时间':<10} {'FSM ID':<8} {'FSM 模式':<12} {'平衡模式':<10} {'状态描述'}")
    print("=" * 80)
    
    start_time = time.time()
    last_fsm_id = None
    
    try:
        while time.time() - start_time < duration:
            try:
                # 获取FSM ID
                fsm_data = sport_client.GetFsmId()
                current_fsm_id = parse_fsm_data(fsm_data)
                
                # 获取FSM模式
                fsm_mode_data = sport_client.GetFsmMode()
                current_fsm_mode = parse_fsm_data(fsm_mode_data)
                
                # 获取平衡模式
                balance_data = sport_client.GetBalanceMode()
                current_balance_mode = parse_fsm_data(balance_data)
                
                # 格式化时间
                current_time = time.strftime("%H:%M:%S")
                
                # 获取状态描述
                status_desc = get_fsm_mode_name(current_fsm_id)
                
                # 打印状态信息
                print(f"{current_time:<10} {current_fsm_id:<8} {current_fsm_mode:<12} {current_balance_mode:<10} {status_desc}")
                
                # 检测状态变化
                if last_fsm_id is not None and last_fsm_id != current_fsm_id:
                    print(f"🔄 状态变化: {get_fsm_mode_name(last_fsm_id)} → {status_desc}")
                
                last_fsm_id = current_fsm_id
                
            except Exception as e:
                print(f"⚠️  获取状态失败: {e}")
            
            time.sleep(1)  # 每秒检查一次
            
    except KeyboardInterrupt:
        print("\n\n⚠️  监控被用户中断")
    
    print("\n📋 监控结束")


def test_fsm_operations(iface="eth0"):
    """
    测试各种FSM操作
    """
    print(f"🧪 测试FSM操作 (接口: {iface})...")
    
    # 初始化
    ChannelFactoryInitialize(0, iface)
    sport_client = LocoClient()
    sport_client.SetTimeout(5.0)
    sport_client.Init()
    
    def show_current_status():
        """显示当前状态"""
        try:
            fsm_id = parse_fsm_data(sport_client.GetFsmId())
            fsm_mode = parse_fsm_data(sport_client.GetFsmMode())
            balance_mode = parse_fsm_data(sport_client.GetBalanceMode())
            status_desc = get_fsm_mode_name(fsm_id)
            
            print(f"当前状态: FSM ID={fsm_id}, 模式={fsm_mode}, 平衡={balance_mode}")
            print(f"状态描述: {status_desc}")
            return fsm_id
        except Exception as e:
            print(f"获取状态失败: {e}")
            return None
    
    print("\n1. 检查初始状态:")
    initial_fsm = show_current_status()
    
    if initial_fsm == 200:
        print("⚠️  机器人已在主运控模式，建议先切换到其他模式测试")
        return
    
    print("\n2. 测试阻尼模式:")
    sport_client.Damp()
    time.sleep(2)
    show_current_status()
    
    print("\n3. 测试站立模式:")
    sport_client.StandUp()
    time.sleep(2)
    show_current_status()
    
    print("\n4. 恢复到初始状态:")
    if initial_fsm is not None:
        sport_client.SetFsmId(initial_fsm)
        time.sleep(2)
        show_current_status()


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="G1机器人FSM状态监控工具")
    parser.add_argument("--iface", default="eth0", help="网络接口名称")
    parser.add_argument("--mode", choices=["monitor", "test"], default="monitor",
                        help="运行模式: monitor(监控) 或 test(测试)")
    parser.add_argument("--duration", type=int, default=30,
                        help="监控持续时间(秒)")
    
    args = parser.parse_args()
    
    try:
        if args.mode == "monitor":
            monitor_robot_status(args.iface, args.duration)
        elif args.mode == "test":
            test_fsm_operations(args.iface)
    except Exception as e:
        print(f"❌ 程序执行失败: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()