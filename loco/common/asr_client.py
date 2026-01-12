import requests

# 配置
ASR_SERVER_URL = "http://192.168.77.103:28003/recognize_live"

class ASRClient:
    """HTTP ASR 客户端（支持固定时长和 VAD 模式）"""
    
    @staticmethod
    def recognize_live(duration=5.0, wait_time=None):
        """
        调用 ASR 服务进行实时录音识别
        
        Args:
            duration: 录音/等待时长(秒)。默认为 5.0。
            wait_time: 显式指定等待时间 (秒)。如果提供，将覆盖 duration 用于控制等待时长。
            
        Returns:
            识别文本
        """
        try:
            # 确保 duration 有值，防止服务端异常
            if duration is None:
                duration = 5.0
                
            payload = {
                "duration": duration,
                "wait_time": wait_time
            }
            
            # 计算超时时间
            base_time = wait_time if wait_time is not None else duration
            timeout = base_time + 5.0
            
            response = requests.post(ASR_SERVER_URL, json=payload, timeout=timeout)
            
            if response.status_code == 200:
                result = response.json()
                if result.get("success"):
                    method = result.get("method", "unknown")
                    print(f"ℹ️ 识别模式: {method}")
                    return result.get("text", "")
                else:
                    print(f"⚠️ ASR 识别失败: {result.get('error')}")
                    return ""
            else:
                print(f"⚠️ ASR 服务请求失败: HTTP {response.status_code}")
                return ""
                
        except requests.exceptions.Timeout:
            print("❌ ASR 服务超时")
            return ""
        except requests.exceptions.ConnectionError:
            print("❌ 无法连接到 ASR 服务，请确保服务已启动")
            return ""
        except Exception as e:
            print(f"❌ ASR 调用异常: {e}")
            return ""