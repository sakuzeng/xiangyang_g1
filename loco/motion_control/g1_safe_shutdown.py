"""
Unitree G1 机器人安全关机程序 - 主运控模式到零力矩模式

本脚本实现 G1 机器人从主运控模式安全切换到零力矩模式的流程：
1. 从主运控模式切换到阻尼模式
2. 从阻尼模式切换到零力矩模式

⚠️ 安全警告：执行此程序前，机器人必须处于悬挂状态！
机器人在零力矩模式下会失去所有关节控制，如果不悬挂会直接倒下。

使用方法:
    python3 g1_safe_shutdown.py <network_interface>

示例:
    python3 g1_safe_shutdown.py eth0
"""
import time
import sys
import json
from typing import Optional
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


def get_mode(val) -> Optional[int]:
    """
    解析运动控制客户端返回的值，提取模式（mode）或状态ID（FSM ID）。
    返回值可能是 JSON 字符串或字典。
    
    参考自 hanger_boot_sequence.py 的实现。
    
    Args:
        val: 来自 LocoClient 的返回值
        
    Returns:
        Optional[int]: 解析出的整数值，如果解析失败则返回 None
    """
    if isinstance(val, str):
        try:
            val = json.loads(val)
        except Exception:
            pass
    if isinstance(val, dict) and "data" in val:
        return int(val["data"])
    try:
        return int(val)
    except (ValueError, TypeError):
        return None


def show_status(loco_client: LocoClient, tag: str) -> None:
    """
    显示当前机器人状态，参考 hanger_boot_sequence.py 的 show 函数
    
    Args:
        loco_client: LocoClient实例
        tag: 状态标签
    """
    fsm_id = get_mode(loco_client.GetFsmId())
    fsm_mode = get_mode(loco_client.GetFsmMode())
    balance_mode = get_mode(loco_client.GetBalanceMode())
    print(f"{tag:<12} → FSM {fsm_id}   mode {fsm_mode}   balance {balance_mode}")


def check_fsm_status(loco_client: LocoClient, expected_fsm_id: int, description: str) -> bool:
    """
    检查当前FSM状态是否符合预期
    
    Args:
        loco_client: LocoClient实例
        expected_fsm_id: 期望的FSM ID
        description: 状态描述
    
    Returns:
        bool: 是否符合预期状态
    """
    current_fsm_id = get_mode(loco_client.GetFsmId())
    
    if current_fsm_id == expected_fsm_id:
        print(f"✅ 成功进入{description}模式 (FSM ID: {expected_fsm_id})")
        return True
    else:
        print(f"❌ 未能进入{description}模式，当前 FSM ID: {current_fsm_id}, 期望: {expected_fsm_id}")
        return False


def safe_shutdown_sequence(loco_client: LocoClient):
    """
    执行安全关机序列：主运控模式 -> 阻尼模式 -> 零力矩模式
    
    Args:
        loco_client: LocoClient实例
    """
    print("=" * 60)
    print("🚀 开始执行 G1 机器人安全关机序列")
    print("=" * 60)
    
    # 步骤1: 检查当前状态
    print("\n📋 步骤1: 检查当前机器人状态...")
    show_status(loco_client, "initial")
    
    current_fsm_id = get_mode(loco_client.GetFsmId())
    if current_fsm_id != 500:
        print(f"⚠️  警告：当前机器人不在主运控模式 (FSM ID: 500)，当前 FSM ID: {current_fsm_id}")
        user_input = input("是否继续执行关机序列？ (y/Y/N): ")
        if user_input.lower() != 'y':
            print("❌ 用户取消操作")
            return False
    
    # 步骤2: 悬挂状态确认（在进入阻尼模式之前）
    print("\n⚠️  步骤2: 悬挂状态安全确认")
    print("=" * 50)
    print("🔴 重要安全警告：即将进入阻尼模式！")
    print("🔴 请确认机器人已经处于以下安全状态之一：")
    print("   1. 机器人被悬挂固定,吊机绳子已接近张紧")
    print("   2. 机器人处在安全的支撑架上")
    print("   3. 周围有足够的安全空间且无人员")
    print("🔴 如果机器人未悬挂，在阻尼模式下可能会失去平衡！")
    print("=" * 50)
    
    hang_confirmation = input("确认机器人已处于悬挂状态或安全支撑状态？(输入 'y' 或 'Y' 继续): ")
    if hang_confirmation.lower() != "y":
        print("❌ 用户未确认悬挂状态，中止操作")
        print("💡 请确保机器人处于安全状态后重新运行程序")
        return False
    
    # 步骤3: 切换到阻尼模式
    print("\n🔄 步骤3: 从主运控模式切换到阻尼模式...")
    print("正在切换到阻尼模式 (FSM ID: 1)...")
    
    loco_client.Damp()  # 设置 FSM ID 为 1
    time.sleep(2)  # 等待状态稳定
    show_status(loco_client, "Damp")
    
    # 验证阻尼模式
    if not check_fsm_status(loco_client, 1, "阻尼"):
        print("❌ 切换到阻尼模式失败，中止操作")
        return False
    
    print("✅ 成功进入阻尼模式，机器人关节已变为阻尼状态")
    print("💡 观察机器人状态：关节应有阻力但不主动运动")
    
    # 步骤4: 最终零力矩确认
    print("\n⚠️  步骤4: 最终零力矩确认")
    print("=" * 50)
    print("🔴 危险警告：即将进入零力矩模式！")
    print("🔴 在零力矩模式下，机器人将失去所有关节控制力！")
    print("🔴 请再次确认机器人已经处于悬挂状态！")
    print("=" * 50)
    
    final_confirmation = input("最终确认机器人已悬挂且可以安全进入零力矩模式？(输入 'y' 或 'Y' 继续): ")
    if final_confirmation.lower() != "y":
        print("❌ 用户未确认最终安全状态，中止零力矩切换")
        print("💡 机器人将保持在阻尼模式")
        return False
    
    # 步骤5: 切换到零力矩模式
    print("\n🔄 步骤5: 从阻尼模式切换到零力矩模式...")
    print("正在切换到零力矩模式 (FSM ID: 0)...")
    
    loco_client.ZeroTorque()  # 设置 FSM ID 为 0
    time.sleep(2)  # 等待状态稳定
    show_status(loco_client, "ZeroTorque")
    
    # 验证零力矩模式
    if not check_fsm_status(loco_client, 0, "零力矩"):
        print("❌ 切换到零力矩模式失败")
        return False
    
    print("✅ 成功进入零力矩模式")
    print("🔴 机器人现在处于零力矩状态，所有关节无控制力")
    
    return True


def main():
    """主函数"""
    # 检查命令行参数，如果没有提供则使用默认值 eth0
    if len(sys.argv) < 2:
        print("⚠️  未指定网络接口，使用默认接口: eth0")
        network_interface = "eth0"
    else:
        network_interface = sys.argv[1]
    
    print(f"🔧 使用网络接口: {network_interface}")
    
    # 程序启动时的总体安全警告
    print("=" * 80)
    print("⚠️  重要安全警告 - G1 机器人安全关机程序 ⚠️")
    print("=" * 80)
    print("🔴 本程序将执行以下操作序列：")
    print("   主运控模式 → 阻尼模式 → 零力矩模式")
    print("🔴 在零力矩模式下，机器人将失去所有关节控制")
    print("🔴 程序执行过程中会多次确认机器人悬挂状态")
    print("🔴 请确保在整个过程中机器人都处于安全悬挂状态")
    print("=" * 80)
    
    initial_safety_check = input("理解安全警告并准备开始？ (输入 'START' 或 'start' 继续): ")
    if initial_safety_check.upper() != "START":
        print("❌ 用户未确认开始，程序退出")
        sys.exit(-1)
    
    try:
        # 初始化DDS通信
        print("\n🔧 初始化DDS通信...")
        ChannelFactoryInitialize(0, network_interface)
        
        # 初始化机器人控制客户端
        print("🔧 初始化LocoClient...")
        loco_client = LocoClient()
        loco_client.SetTimeout(10.0)
        loco_client.Init()
        
        # 执行安全关机序列
        success = safe_shutdown_sequence(loco_client)
        
        if success:
            print("\n" + "=" * 60)
            print("✅ 安全关机序列执行成功")
            print("🔴 机器人现在处于零力矩模式")
            print("💡 如需重新启动机器人，请先将机器人切换到安全模式")
            print("💡 可以使用 hanger_boot_sequence.py 进行重新启动")
            print("=" * 60)
        else:
            print("\n" + "=" * 60)
            print("❌ 安全关机序列执行失败")
            print("💡 请检查机器人状态并重试")
            print("=" * 60)
    
    except KeyboardInterrupt:
        print("\n\n❌ 用户中断程序")
        print("💡 如果机器人状态异常，请手动检查并恢复")
    except Exception as e:
        print(f"\n❌ 程序执行出错: {e}")
        print("💡 请检查网络连接和机器人状态")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()