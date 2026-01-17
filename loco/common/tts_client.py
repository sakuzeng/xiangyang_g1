import time
import requests
import logging
from .logger import setup_logger

# 配置日志
logger = setup_logger("tts_client")

# 配置
TTS_SERVER_URL = "http://192.168.77.103:28001/speak_msg"
TTS_MONITOR_URL = "http://192.168.77.103:28001/monitor"
TTS_EXCLUSIVE_MODE_URL = "http://192.168.77.103:28001/control/exclusive_mode"
TTS_STOP_CURRENT_PLAY_URL = "http://192.168.77.103:28001/control/stop_current_playback"
class TTSClient:
    """HTTP TTS 客户端"""
    DEFAULT_SOURCE = "emergency_call"
    
    @staticmethod
    def set_exclusive_mode(active: bool, allowed_source: str = None, max_wait_seconds=3):
        """
        控制语音服务的独占模式
        
        Args:
            active: True 开启独占，False 关闭独占
            allowed_source: 独占时的允许源
            max_wait_seconds: 获取独占权的最大等待时间
        """
        if allowed_source is None:
            allowed_source = TTSClient.DEFAULT_SOURCE

        try:
            payload = {
                "active": active,
                "allowed_source": allowed_source
            }
            
            if active:
                start_time = time.time()
                attempt = 0
                
                while time.time() - start_time < max_wait_seconds:
                    attempt += 1
                    response = requests.post(TTS_EXCLUSIVE_MODE_URL, json=payload, timeout=2.0)
                    
                    if response.status_code == 200:
                        data = response.json()
                        is_granted = data.get("is_granted", False)
                        
                        if is_granted:
                            logger.info(f"✅ [{allowed_source}] 成功获得TTS独占权 (第{attempt}次尝试)")
                            return True
                        else:
                            current_source = data.get("current_source")
                            logger.warning(f"⚠️ [{allowed_source}] 等待独占权... (当前持有者: {current_source})")
                            time.sleep(0.3)
                    else:
                        logger.warning(f"⚠️ 设置TTS独占模式请求失败: HTTP {response.status_code}")
                        return False
                
                logger.error(f"❌ [{allowed_source}] 获取独占权超时 ({max_wait_seconds}秒)")
                return False
            else:
                response = requests.post(TTS_EXCLUSIVE_MODE_URL, json=payload, timeout=2.0)
                if response.status_code == 200:
                    data = response.json()
                    if data.get("is_granted", False):
                        logger.info(f"🔓 [{allowed_source}] TTS独占模式已释放")
                        return True
                    else:
                        logger.warning(f"⚠️ [{allowed_source}] 释放独占模式失败: {data.get('message', '未知错误')}")
                        return False
                return False
                
        except Exception as e:
            logger.error(f"⚠️ 设置TTS独占模式异常: {e}")
            return False
            
    @staticmethod
    def stop_current_playback(source=None):
        """停止当前播放 (保留队列)
        Args:
            source: 请求停止的来源，用于独占模式校验。如果不传，默认为 DEFAULT_SOURCE
        """
        if source is None:
            source = TTSClient.DEFAULT_SOURCE

        try:
            payload = {
                "allowed_source": source
            }

            response = requests.post(TTS_STOP_CURRENT_PLAY_URL, json=payload, timeout=2.0)
            if response.status_code == 200:
                logger.info(f"🛑 [{source}] 已发送停止当前播放请求")
                return True
            else:
                logger.warning(f"⚠️ 停止当前播放失败: {response.status_code}")
                return False
        except Exception as e:
            logger.error(f"❌ 停止当前播放请求异常: {e}")
            return False

    @staticmethod
    def speak(text, volume=100, wait=True, source=None):
        """发送TTS请求并可选等待播放完成"""
        if not text:
            return None
        
        if source is None:
            source = TTSClient.DEFAULT_SOURCE

        try:
            payload = {
                "speak_msg": text,
                "source": source,
                "volume": volume
            }
            headers = {"Content-Type": "application/json"}
            
            logger.info(f"🔊 {text}")
            # 增加超时时间，防止长文本请求超时
            response = requests.post(TTS_SERVER_URL, json=payload, headers=headers, timeout=10.0)
            
            if response.status_code != 200:
                logger.warning(f"⚠️ TTS错误: {response.status_code}")
                return None
            
            result = response.json()
            data = result.get('data')
            if not data or not isinstance(data, dict):
                return None
            
            task_id = data.get('task_id')
            
            if wait and task_id:
                TTSClient._wait_for_completion(task_id)
            
            return task_id
                
        except Exception as e:
            logger.error(f"❌ TTS失败: {e}")
            return None

    @staticmethod
    def is_task_running(task_id):
        """检查任务是否正在运行 (非阻塞)"""
        if not task_id:
            return False
            
        try:
            response = requests.get(TTS_MONITOR_URL, timeout=0.5)
            if response.status_code == 200:
                data = response.json()
                active_task = data.get('active_task')
                waiting_list = data.get('waiting_list', [])
                
                if active_task and active_task.get('id') == task_id:
                    return True
                
                for t in waiting_list:
                    if t.get('id') == task_id:
                        return True
                
                return False
        except:
            pass
        return False

    @staticmethod
    def _wait_for_completion(task_id, timeout=120):
        """等待任务完成 (基于任务不在活动与队列中)"""
        start_time = time.time()
        check_interval = 0.05 # ⚡ 优化: 缩短轮询间隔，提高响应速度 (原0.2s)
        task_seen = False
        stable_checks = 0
        REQUIRED_STABLE_CHECKS = 5  # 增加到 5 次 (1秒)，防止任务在网关转发间隙“闪烁”导致误判
        
        # 新增: 等待任务出现的最大时间。如果超过此时间任务仍未出现，假定任务已完成(过快)或失败
        MAX_STARTUP_WAIT = 5.0 
        
        while time.time() - start_time < timeout:
            try:
                # 复用 is_task_running 判断任务是否存在
                if TTSClient.is_task_running(task_id):
                    task_seen = True
                    stable_checks = 0
                else:
                    if task_seen:
                        stable_checks += 1
                        if stable_checks >= REQUIRED_STABLE_CHECKS:
                            return True
                    elif time.time() - start_time > MAX_STARTUP_WAIT:
                        # 任务长时间未出现，假定已结束
                        return True
            except:
                pass
            time.sleep(check_interval)
        return False

    @staticmethod
    def check_exclusive_ownership():
        """检查是否仍持有独占权"""
        try:
            response = requests.get(TTS_MONITOR_URL, timeout=0.5)
            if response.status_code == 200:
                data = response.json()
                exclusive_mode = data.get("exclusive_mode", {})
                current_source = exclusive_mode.get("source")
                
                if current_source == TTSClient.DEFAULT_SOURCE:
                    return True
                else:
                    return False
        except:
            pass
        return False
