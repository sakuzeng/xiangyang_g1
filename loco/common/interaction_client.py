import requests
import time

# 配置
INTERACTION_API = "http://192.168.77.103:28004"

class InteractionClient:
    """Interaction 服务客户端（管理唤醒检测）"""
    
    @staticmethod
    def pause_wake(source="demo"):
        """
        暂停 interaction 的唤醒检测
        
        Args:
            source: 调用来源标识
            
        Returns:
            bool: 成功返回 True
        """
        try:
            response = requests.post(
                f"{INTERACTION_API}/wake/pause",
                json={"source": source},
                timeout=2.0
            )
            if response.status_code == 200:
                data = response.json()
                success = data.get("success", False)
                if success:
                    print(f"⏸️  已暂停唤醒检测 (来源: {source})")
                return success
        except requests.exceptions.ConnectionError:
            print("⚠️ 无法连接到 interaction 服务")
        except requests.exceptions.Timeout:
            print("⚠️ interaction 服务请求超时")
        except Exception as e:
            print(f"⚠️ 暂停唤醒检测异常: {e}")
        return False
    
    @staticmethod
    def resume_wake(source="demo"):
        """
        恢复 interaction 的唤醒检测
        
        Args:
            source: 调用来源标识
            
        Returns:
            bool: 成功返回 True
        """
        try:
            response = requests.post(
                f"{INTERACTION_API}/wake/resume",
                json={"source": source},
                timeout=2.0
            )
            if response.status_code == 200:
                data = response.json()
                success = data.get("success", False)
                if success:
                    print(f"▶️  已恢复唤醒检测 (来源: {source})")
                return success
        except requests.exceptions.ConnectionError:
            print("⚠️ 无法连接到 interaction 服务")
        except requests.exceptions.Timeout:
            print("⚠️ interaction 服务请求超时")
        except Exception as e:
            print(f"⚠️ 恢复唤醒检测异常: {e}")
        return False
    
    @staticmethod
    def get_wake_status():
        """
        获取当前唤醒检测状态
        
        Returns:
            dict: {"active": bool, "paused_by": str} 或 None
        """
        try:
            response = requests.get(
                f"{INTERACTION_API}/wake/status",
                timeout=2.0
            )
            if response.status_code == 200:
                data = response.json()
                if data.get("success"):
                    return data.get("data")
        except:
            pass
        return None


class WakeControl:
    """
    上下文管理器：自动管理唤醒检测的暂停和恢复
    
    用法:
        with WakeControl(source="my_demo"):
            # 在这个代码块内，唤醒检测会被暂停
            do_something()
        # 退出代码块后自动恢复
    """
    
    def __init__(self, source="demo", pause_duration=None):
        """
        Args:
            source: 调用来源标识
            pause_duration: 暂停时长(秒)，None 表示手动控制
        """
        self.source = source
        self.pause_duration = pause_duration
        self._paused = False
    
    def __enter__(self):
        """进入上下文时暂停唤醒检测"""
        self._paused = InteractionClient.pause_wake(self.source)
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """退出上下文时恢复唤醒检测"""
        if self._paused:
            if self.pause_duration:
                time.sleep(self.pause_duration)
            InteractionClient.resume_wake(self.source)
        return False  # 不抑制异常