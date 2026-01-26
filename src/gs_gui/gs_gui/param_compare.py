#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of param compare.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
参数对比模块

支持多种对比模式：
- 默认值对比：当前值 vs 出厂默认值
- USV 对比：USV A vs USV B
- 文件对比：当前参数 vs 导入文件

提供差异高亮、过滤、同步等功能
"""

from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
from .param_manager import ParamInfo
from .param_metadata import get_param_metadata


class CompareMode(Enum):
    """对比模式"""
    DEFAULT = "default"      # 当前值 vs 默认值
    USV = "usv"             # USV A vs USV B
    FILE = "file"           # 当前值 vs 文件


class DiffType(Enum):
    """差异类型"""
    SAME = "same"           # 相同
    DIFFERENT = "different" # 不同
    MISSING_LEFT = "missing_left"   # 左侧缺失
    MISSING_RIGHT = "missing_right" # 右侧缺失


@dataclass
class ParamDiff:
    """参数差异"""
    param_name: str
    left_value: Optional[float]
    right_value: Optional[float]
    diff_type: DiffType
    left_label: str = "当前值"
    right_label: str = "默认值"
    
    @property
    def diff_percent(self) -> Optional[float]:
        """差异百分比"""
        if self.left_value is None or self.right_value is None:
            return None
        if self.right_value == 0:
            return None
        return abs((self.left_value - self.right_value) / self.right_value) * 100
    
    @property
    def diff_absolute(self) -> Optional[float]:
        """绝对差异"""
        if self.left_value is None or self.right_value is None:
            return None
        return abs(self.left_value - self.right_value)


class ParamCompare:
    """
    参数对比工具
    
    提供多种对比功能和差异分析
    """
    
    @staticmethod
    def compare_with_default(params: Dict[str, ParamInfo]) -> List[ParamDiff]:
        """
        对比当前值和默认值
        
        Args:
            params: 参数字典
            
        Returns:
            差异列表
        """
        diffs = []
        
        for name, param in params.items():
            meta = get_param_metadata(name)
            if not meta or meta.default_value is None:
                continue
            
            # 判断差异类型
            if abs(param.value - meta.default_value) < 1e-9:
                diff_type = DiffType.SAME
            else:
                diff_type = DiffType.DIFFERENT
            
            diff = ParamDiff(
                param_name=name,
                left_value=param.value,
                right_value=meta.default_value,
                diff_type=diff_type,
                left_label="当前值",
                right_label="默认值"
            )
            diffs.append(diff)
        
        return diffs
    
    @staticmethod
    def compare_two_param_sets(
        left_params: Dict[str, ParamInfo],
        right_params: Dict[str, ParamInfo],
        left_label: str = "左侧",
        right_label: str = "右侧"
    ) -> List[ParamDiff]:
        """
        对比两组参数
        
        Args:
            left_params: 左侧参数字典
            right_params: 右侧参数字典
            left_label: 左侧标签
            right_label: 右侧标签
            
        Returns:
            差异列表
        """
        diffs = []
        
        # 所有参数名称（去重）
        all_param_names = set(left_params.keys()) | set(right_params.keys())
        
        for name in sorted(all_param_names):
            left_param = left_params.get(name)
            right_param = right_params.get(name)
            
            # 判断差异类型
            if left_param is None:
                diff_type = DiffType.MISSING_LEFT
                left_value = None
                right_value = right_param.value
            elif right_param is None:
                diff_type = DiffType.MISSING_RIGHT
                left_value = left_param.value
                right_value = None
            elif abs(left_param.value - right_param.value) < 1e-9:
                diff_type = DiffType.SAME
                left_value = left_param.value
                right_value = right_param.value
            else:
                diff_type = DiffType.DIFFERENT
                left_value = left_param.value
                right_value = right_param.value
            
            diff = ParamDiff(
                param_name=name,
                left_value=left_value,
                right_value=right_value,
                diff_type=diff_type,
                left_label=left_label,
                right_label=right_label
            )
            diffs.append(diff)
        
        return diffs
    
    @staticmethod
    def filter_diffs(
        diffs: List[ParamDiff],
        show_same: bool = False,
        show_different: bool = True,
        show_missing: bool = True,
        min_diff_percent: Optional[float] = None
    ) -> List[ParamDiff]:
        """
        过滤差异列表
        
        Args:
            diffs: 差异列表
            show_same: 是否显示相同项
            show_different: 是否显示不同项
            show_missing: 是否显示缺失项
            min_diff_percent: 最小差异百分比（仅显示大于此值的）
            
        Returns:
            过滤后的差异列表
        """
        filtered = []
        
        for diff in diffs:
            # 类型过滤
            if diff.diff_type == DiffType.SAME and not show_same:
                continue
            if diff.diff_type == DiffType.DIFFERENT and not show_different:
                continue
            if diff.diff_type in (DiffType.MISSING_LEFT, DiffType.MISSING_RIGHT) and not show_missing:
                continue
            
            # 百分比过滤
            if min_diff_percent is not None:
                if diff.diff_percent is None or diff.diff_percent < min_diff_percent:
                    continue
            
            filtered.append(diff)
        
        return filtered
    
    @staticmethod
    def get_statistics(diffs: List[ParamDiff]) -> Dict[str, int]:
        """
        获取差异统计
        
        Args:
            diffs: 差异列表
            
        Returns:
            统计字典
        """
        stats = {
            "total": len(diffs),
            "same": 0,
            "different": 0,
            "missing_left": 0,
            "missing_right": 0
        }
        
        for diff in diffs:
            if diff.diff_type == DiffType.SAME:
                stats["same"] += 1
            elif diff.diff_type == DiffType.DIFFERENT:
                stats["different"] += 1
            elif diff.diff_type == DiffType.MISSING_LEFT:
                stats["missing_left"] += 1
            elif diff.diff_type == DiffType.MISSING_RIGHT:
                stats["missing_right"] += 1
        
        return stats
    
    @staticmethod
    def get_top_diffs(
        diffs: List[ParamDiff],
        top_n: int = 10,
        by: str = "percent"
    ) -> List[ParamDiff]:
        """
        获取差异最大的前 N 个参数
        
        Args:
            diffs: 差异列表
            top_n: 返回数量
            by: 排序依据（"percent" 或 "absolute"）
            
        Returns:
            差异最大的参数列表
        """
        # 只考虑不同的参数
        different_diffs = [d for d in diffs if d.diff_type == DiffType.DIFFERENT]
        
        if by == "percent":
            # 按百分比排序
            sorted_diffs = sorted(
                different_diffs,
                key=lambda d: d.diff_percent if d.diff_percent is not None else 0,
                reverse=True
            )
        else:  # absolute
            # 按绝对值排序
            sorted_diffs = sorted(
                different_diffs,
                key=lambda d: d.diff_absolute if d.diff_absolute is not None else 0,
                reverse=True
            )
        
        return sorted_diffs[:top_n]
    
    @staticmethod
    def generate_sync_script(
        diffs: List[ParamDiff],
        direction: str = "left_to_right"
    ) -> List[Tuple[str, float]]:
        """
        生成同步脚本（将一侧的值同步到另一侧）
        
        Args:
            diffs: 差异列表
            direction: 同步方向（"left_to_right" 或 "right_to_left"）
            
        Returns:
            需要修改的参数列表 [(param_name, new_value), ...]
        """
        sync_list = []
        
        for diff in diffs:
            if diff.diff_type != DiffType.DIFFERENT:
                continue
            
            if direction == "left_to_right":
                # 将左侧值同步到右侧
                if diff.left_value is not None:
                    sync_list.append((diff.param_name, diff.left_value))
            else:  # right_to_left
                # 将右侧值同步到左侧
                if diff.right_value is not None:
                    sync_list.append((diff.param_name, diff.right_value))
        
        return sync_list
