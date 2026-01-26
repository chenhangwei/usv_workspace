#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of param search.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
高级搜索模块

支持：
- 参数名称搜索
- 描述搜索
- 正则表达式
- 多条件过滤（范围、分组、修改状态）
"""

import re
from typing import Dict, List, Optional
from .param_manager import ParamInfo
from .param_metadata import get_param_metadata


class AdvancedSearch:
    """高级搜索工具"""
    
    @staticmethod
    def search_by_name(
        params: Dict[str, ParamInfo],
        keyword: str,
        case_sensitive: bool = False
    ) -> List[str]:
        """按名称搜索"""
        if not case_sensitive:
            keyword = keyword.lower()
        
        results = []
        for name in params.keys():
            search_name = name if case_sensitive else name.lower()
            if keyword in search_name:
                results.append(name)
        
        return sorted(results)
    
    @staticmethod
    def search_by_description(
        params: Dict[str, ParamInfo],
        keyword: str,
        case_sensitive: bool = False
    ) -> List[str]:
        """按描述搜索"""
        if not case_sensitive:
            keyword = keyword.lower()
        
        results = []
        for name, param in params.items():
            # 从元数据获取描述
            meta = get_param_metadata(name)
            description = meta.description if meta else param.description
            
            search_desc = description if case_sensitive else description.lower()
            if keyword in search_desc:
                results.append(name)
        
        return sorted(results)
    
    @staticmethod
    def search_by_regex(
        params: Dict[str, ParamInfo],
        pattern: str,
        search_in: str = "name"  # "name" or "description"
    ) -> List[str]:
        """正则表达式搜索"""
        try:
            regex = re.compile(pattern, re.IGNORECASE)
        except re.error:
            return []
        
        results = []
        for name, param in params.items():
            if search_in == "name":
                if regex.search(name):
                    results.append(name)
            else:  # description
                meta = get_param_metadata(name)
                description = meta.description if meta else param.description
                if regex.search(description):
                    results.append(name)
        
        return sorted(results)
    
    @staticmethod
    def filter_by_group(
        params: Dict[str, ParamInfo],
        group: str
    ) -> List[str]:
        """按分组过滤"""
        results = []
        for name, param in params.items():
            if param.group == group:
                results.append(name)
        
        return sorted(results)
    
    @staticmethod
    def filter_by_modified(
        params: Dict[str, ParamInfo],
        modified_only: bool = True
    ) -> List[str]:
        """过滤修改的参数"""
        results = []
        for name, param in params.items():
            if modified_only:
                if param.is_modified:
                    results.append(name)
            else:
                if not param.is_modified:
                    results.append(name)
        
        return sorted(results)
    
    @staticmethod
    def filter_by_value_range(
        params: Dict[str, ParamInfo],
        min_value: Optional[float] = None,
        max_value: Optional[float] = None
    ) -> List[str]:
        """按值范围过滤"""
        results = []
        for name, param in params.items():
            if min_value is not None and param.value < min_value:
                continue
            if max_value is not None and param.value > max_value:
                continue
            results.append(name)
        
        return sorted(results)
    
    @staticmethod
    def combined_search(
        params: Dict[str, ParamInfo],
        name_keyword: Optional[str] = None,
        desc_keyword: Optional[str] = None,
        group: Optional[str] = None,
        modified_only: Optional[bool] = None,
        min_value: Optional[float] = None,
        max_value: Optional[float] = None
    ) -> List[str]:
        """组合搜索（多条件AND）"""
        result_sets = []
        
        # 按名称搜索
        if name_keyword:
            result_sets.append(set(AdvancedSearch.search_by_name(params, name_keyword)))
        
        # 按描述搜索
        if desc_keyword:
            result_sets.append(set(AdvancedSearch.search_by_description(params, desc_keyword)))
        
        # 按分组过滤
        if group:
            result_sets.append(set(AdvancedSearch.filter_by_group(params, group)))
        
        # 按修改状态过滤
        if modified_only is not None:
            result_sets.append(set(AdvancedSearch.filter_by_modified(params, modified_only)))
        
        # 按值范围过滤
        if min_value is not None or max_value is not None:
            result_sets.append(set(AdvancedSearch.filter_by_value_range(params, min_value, max_value)))
        
        # 如果没有条件，返回所有
        if not result_sets:
            return sorted(params.keys())
        
        # 取交集
        final_results = result_sets[0]
        for result_set in result_sets[1:]:
            final_results &= result_set
        
        return sorted(list(final_results))
