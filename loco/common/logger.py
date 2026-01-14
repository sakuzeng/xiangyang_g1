import logging
import time
import sys

class LocalFormatter(logging.Formatter):
    """
    自定义日志格式化器，支持毫秒级时间戳
    """
    def formatTime(self, record, datefmt=None):
        ct = self.converter(record.created)
        if datefmt:
            s = time.strftime(datefmt, ct)
        else:
            t = time.strftime("%Y-%m-%d %H:%M:%S", ct)
            s = "%s,%03d" % (t, record.msecs)
        return s

def setup_logger(name):
    """
    配置并获取一个支持 UTC+8 时间格式的 logger
    """
    # 配置日志时间转换为 UTC+8(docker中的时间不对)
    # logging.Formatter.converter = lambda *args: time.localtime(time.time())
    
    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)
    # 防止日志向上传递给 root logger，避免重复打印
    logger.propagate = False

    # 避免重复添加 handler
    if not logger.handlers:
        handler = logging.StreamHandler(sys.stdout)
        formatter = LocalFormatter(
            # 修改格式，包含 Logger 名称，模仿 INFO:core: 风格但带时间戳
            "[%(asctime)s] %(levelname)s:%(name)s: %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S"
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)
    
    return logger