import sys
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

def main():
    """LED控制程序 - 从不亮到蓝色"""
    
    # 检查命令行参数
    if len(sys.argv) < 2:
        print("未提供网络接口名称，使用默认值: eth0")
        interface_name = "eth0"
    else:
        interface_name = sys.argv[1]
        print(f"使用网络接口: {interface_name}")
    
    # interface_name = sys.argv[]
    print(f"使用网络接口: {interface_name}")
    
    # 初始化通道
    ChannelFactoryInitialize(0, interface_name)
    
    # 创建音频客户端(用于控制LED)
    audio_client = AudioClient()
    audio_client.SetTimeout(10.0)
    audio_client.Init()
    
    print("=== LED控制程序 ===")
    print("从不亮到蓝色\n")
    
    # LED控制序列
    led_colors = [
        (0, 0, 0, "关闭(不亮)"),
        (0, 0, 255, "蓝色")
    ]
    
    for r, g, b, name in led_colors:
        print(f"LED设置为: {name} - RGB({r}, {g}, {b})")
        code = audio_client.LedControl(r, g, b)
        
        if code == 0:
            print(f"✓ LED控制成功")
        else:
            print(f"✗ LED控制失败，错误码: {code}")
        
        time.sleep(2)  # 等待2秒再切换到下一个颜色
    
    print("\nLED控制完成！")

if __name__ == "__main__":
    main()