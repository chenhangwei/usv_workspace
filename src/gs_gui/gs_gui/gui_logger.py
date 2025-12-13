"""
GUI 统一日志模块

为 gs_gui 包提供统一的日志记录器，替代 print 语句。
支持控制台输出和文件输出。
"""

import logging
import os
from datetime import datetime
from pathlib import Path
from typing import Optional


def get_logger(name: str = "gs_gui") -> logging.Logger:
    """
    获取 GUI 日志记录器
    
    Args:
        name: 日志记录器名称，用于区分不同模块
        
    Returns:
        配置好的 Logger 实例
    """
    logger = logging.getLogger(name)
    
    # 如果已配置，直接返回
    if logger.handlers:
        return logger
    
    logger.setLevel(logging.DEBUG)
    
    # 控制台处理器
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_format = logging.Formatter(
        "[%(name)s] [%(levelname)s] %(message)s"
    )
    console_handler.setFormatter(console_format)
    logger.addHandler(console_handler)
    
    return logger


def setup_file_logging(log_dir: Optional[str] = None) -> Optional[Path]:
    """
    设置文件日志（可选）
    
    Args:
        log_dir: 日志目录，默认为 ~/.ros/log/gs_gui/
        
    Returns:
        日志文件路径，失败返回 None
    """
    if log_dir is None:
        log_dir = os.path.expanduser("~/.ros/log/gs_gui")
    
    try:
        log_path = Path(log_dir)
        log_path.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = log_path / f"gs_gui_{timestamp}.log"
        
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.DEBUG)
        file_format = logging.Formatter(
            "%(asctime)s [%(name)s] [%(levelname)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S"
        )
        file_handler.setFormatter(file_format)
        
        # 添加到根 logger
        root_logger = logging.getLogger("gs_gui")
        root_logger.addHandler(file_handler)
        
        return log_file
    except Exception as e:
        print(f"设置文件日志失败: {e}")
        return None


# 预创建常用模块的 logger
logger = get_logger("gs_gui")
