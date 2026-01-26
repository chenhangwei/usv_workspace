#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of error handler.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
统一错误处理模块
提供分类错误处理、自动恢复和日志记录功能
"""

import logging
from enum import Enum
from typing import Callable, Optional, Any
from functools import wraps
import traceback


class ErrorSeverity(Enum):
    """错误严重程度分类"""
    CRITICAL = "critical"  # 系统级错误，需要立即停止
    ERROR = "error"        # 功能错误，需要记录并通知
    WARNING = "warning"    # 警告，可以继续运行
    INFO = "info"          # 信息性消息


class ErrorCategory(Enum):
    """错误类别"""
    NETWORK = "network"              # 网络/通信错误
    HARDWARE = "hardware"            # 硬件错误
    PARAMETER = "parameter"          # 参数错误
    STATE = "state"                  # 状态错误
    TIMEOUT = "timeout"              # 超时错误
    RESOURCE = "resource"            # 资源错误（内存、文件等）
    UNKNOWN = "unknown"              # 未知错误


class ErrorRecoveryStrategy:
    """错误恢复策略"""
    
    @staticmethod
    def retry_with_backoff(func: Callable, max_retries: int = 3, 
                           base_delay: float = 1.0) -> Optional[Any]:
        """
        指数退避重试策略
        
        Args:
            func: 要重试的函数
            max_retries: 最大重试次数
            base_delay: 基础延迟时间（秒）
            
        Returns:
            函数执行结果或 None
        """
        import time
        for attempt in range(max_retries):
            try:
                return func()
            except Exception as e:
                if attempt == max_retries - 1:
                    raise
                delay = base_delay * (2 ** attempt)
                logging.warning(f"重试 {attempt + 1}/{max_retries}，{delay}秒后重试: {e}")
                time.sleep(delay)
        return None
    
    @staticmethod
    def fallback_value(func: Callable, fallback: Any, 
                       log_error: bool = True) -> Any:
        """
        降级处理策略：返回默认值
        
        Args:
            func: 要执行的函数
            fallback: 失败时返回的默认值
            log_error: 是否记录错误
            
        Returns:
            函数执行结果或默认值
        """
        try:
            return func()
        except Exception as e:
            if log_error:
                logging.error(f"操作失败，返回默认值: {e}")
            return fallback


class RobustErrorHandler:
    """健壮错误处理器"""
    
    def __init__(self, node_logger=None):
        """
        初始化错误处理器
        
        Args:
            node_logger: ROS 节点日志记录器
        """
        self.logger = node_logger or logging.getLogger(__name__)
        self._error_counts = {}  # 错误计数器
        self._error_threshold = 10  # 错误阈值
    
    def handle_error(self, error: Exception, context: str, 
                     severity: ErrorSeverity = ErrorSeverity.ERROR,
                     category: ErrorCategory = ErrorCategory.UNKNOWN,
                     recovery_action: Optional[Callable] = None) -> bool:
        """
        统一错误处理入口
        
        Args:
            error: 异常对象
            context: 错误上下文描述
            severity: 错误严重程度
            category: 错误类别
            recovery_action: 恢复动作回调
            
        Returns:
            是否成功恢复
        """
        # 记录错误
        error_key = f"{category.value}:{context}"
        self._error_counts[error_key] = self._error_counts.get(error_key, 0) + 1
        
        # 构造错误消息
        error_msg = (
            f"[{severity.value.upper()}] [{category.value}] {context}\n"
            f"错误: {type(error).__name__}: {str(error)}\n"
            f"发生次数: {self._error_counts[error_key]}"
        )
        
        # 根据严重程度记录日志
        if severity == ErrorSeverity.CRITICAL:
            self.logger.error(error_msg)
            self.logger.error(f"堆栈跟踪:\n{traceback.format_exc()}")
        elif severity == ErrorSeverity.ERROR:
            self.logger.error(error_msg)
        elif severity == ErrorSeverity.WARNING:
            self.logger.warn(error_msg)
        else:
            self.logger.info(error_msg)
        
        # 检查是否超过错误阈值
        if self._error_counts[error_key] >= self._error_threshold:
            self.logger.error(
                f"错误 {error_key} 超过阈值 {self._error_threshold}，可能需要人工干预"
            )
        
        # 尝试恢复
        if recovery_action:
            try:
                recovery_action()
                self.logger.info(f"恢复动作成功: {context}")
                return True
            except Exception as recovery_error:
                self.logger.error(f"恢复动作失败: {recovery_error}")
                return False
        
        return False
    
    def reset_error_count(self, category: ErrorCategory, context: str):
        """重置特定错误的计数"""
        error_key = f"{category.value}:{context}"
        if error_key in self._error_counts:
            del self._error_counts[error_key]


def safe_execute(error_handler: RobustErrorHandler, 
                 context: str,
                 category: ErrorCategory = ErrorCategory.UNKNOWN,
                 severity: ErrorSeverity = ErrorSeverity.ERROR,
                 recovery_action: Optional[Callable] = None):
    """
    装饰器：为函数提供统一错误处理
    
    Args:
        error_handler: 错误处理器实例
        context: 错误上下文
        category: 错误类别
        severity: 错误严重程度
        recovery_action: 恢复动作
    """
    def decorator(func: Callable):
        @wraps(func)
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                error_handler.handle_error(
                    e, context, severity, category, recovery_action
                )
                # 根据严重程度决定是否重新抛出异常
                if severity == ErrorSeverity.CRITICAL:
                    raise
                return None
        return wrapper
    return decorator


# 使用示例
"""
# 在 GroundStationNode 中初始化
self.error_handler = RobustErrorHandler(self.get_logger())

# 方法1: 直接使用
try:
    # 风险操作
    result = risky_operation()
except ConnectionError as e:
    self.error_handler.handle_error(
        e, 
        context="发送导航目标到 usv_01",
        severity=ErrorSeverity.ERROR,
        category=ErrorCategory.NETWORK,
        recovery_action=lambda: self.reconnect_usv("usv_01")
    )

# 方法2: 使用装饰器
@safe_execute(
    self.error_handler, 
    "处理 USV 状态回调",
    category=ErrorCategory.STATE,
    severity=ErrorSeverity.WARNING
)
def usv_state_callback(self, msg, usv_id):
    # 处理逻辑
    pass
"""
