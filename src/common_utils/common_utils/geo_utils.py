#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of geo utils.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
地理坐标转换工具类

该模块提供 WGS84 椭球模型参数和高精度坐标转换函数。
用于统一 USV 系统中的 XYZ (ENU) 与 GPS (WGS84) 坐标转换，消除多节点间的计算误差。
"""

import math

class GeoUtils:
    """地理坐标转换工具类"""
    
    # WGS84 椭球参数（国际标准）
    WGS84_A = 6378137.0                          # 赤道半径（米）
    WGS84_B = 6356752.314245                     # 极半径（米）
    WGS84_E2 = 1 - (WGS84_B * WGS84_B) / (WGS84_A * WGS84_A)  # 第一偏心率平方

    @staticmethod
    def meters_per_lat_degree(lat: float) -> float:
        """
        计算指定纬度处 1° 纬度对应的弧长（米/度）
        使用 WGS84 椭球模型，精度优于球体近似
        
        Args:
            lat: 纬度（度）
        
        Returns:
            float: 该纬度处 1° 纬度的弧长（米）
        """
        lat_rad = math.radians(lat)
        sin_lat = math.sin(lat_rad)
        
        # WGS84 椭球纬度弧长公式
        # M(φ) = πa(1-e²) / [180(1-e²sin²φ)^(3/2)]
        numerator = math.pi * GeoUtils.WGS84_A * (1 - GeoUtils.WGS84_E2)
        denominator = 180 * math.pow(1 - GeoUtils.WGS84_E2 * sin_lat * sin_lat, 1.5)
        
        return numerator / denominator
    
    @staticmethod
    def meters_per_lon_degree(lat: float) -> float:
        """
        计算指定纬度处 1° 经度对应的弧长（米/度）
        使用 WGS84 椭球模型，精度优于球体近似
        
        Args:
            lat: 纬度（度）
        
        Returns:
            float: 该纬度处 1° 经度的弧长（米）
        """
        lat_rad = math.radians(lat)
        cos_lat = math.cos(lat_rad)
        sin_lat = math.sin(lat_rad)
        
        # WGS84 椭球经度弧长公式
        # N(φ) = πa·cosφ / [180·√(1-e²sin²φ)]
        numerator = math.pi * GeoUtils.WGS84_A * cos_lat
        denominator = 180 * math.sqrt(1 - GeoUtils.WGS84_E2 * sin_lat * sin_lat)
        
        return numerator / denominator

    @staticmethod
    def xyz_to_gps(x: float, y: float, z: float, origin_lat: float, origin_lon: float, origin_alt: float) -> dict:
        """
        将 XYZ 坐标转换为 GPS 坐标 (lat/lon/alt)
        使用 WGS84 椭球模型进行高精度转换
        
        Args:
            x: 东向距离(米)
            y: 北向距离(米)
            z: 高度(米)
            origin_lat: 原点纬度
            origin_lon: 原点经度
            origin_alt: 原点海拔
        
        Returns:
            dict: {'lat': 纬度, 'lon': 经度, 'alt': 海拔}
        """
        # 1. 先计算纬度 (纬度变换基本只与y有关)
        # 使用原点纬度的系数进行初步估算
        meters_per_lat = GeoUtils.meters_per_lat_degree(origin_lat)
        dlat = y / meters_per_lat
        lat = origin_lat + dlat
        
        # 2. 计算经度 (经度变换强烈依赖纬度)
        # 使用 mid_lat = (origin_lat + current_lat) / 2 来计算经度系数
        mid_lat = (origin_lat + lat) / 2.0
        meters_per_lon = GeoUtils.meters_per_lon_degree(mid_lat)
        
        dlon = x / meters_per_lon
        lon = origin_lon + dlon
        
        # 3. 计算高度
        # 高度 = 原点海拔 + Z偏移
        alt = origin_alt + z
        
        return {'lat': lat, 'lon': lon, 'alt': alt}

    @staticmethod
    def gps_to_xyz(lat: float, lon: float, alt: float, origin_lat: float, origin_lon: float, origin_alt: float) -> dict:
        """
        GPS 坐标 → 本地 XYZ (ENU坐标系)
        
        使用 WGS84 椭球模型进行高精度转换
        
        Args:
            lat: 纬度（度）
            lon: 经度（度）
            alt: 海拔（米）
            origin_lat: 原点纬度
            origin_lon: 原点经度
            origin_alt: 原点海拔
        
        Returns:
            {'x': 东向距离(m), 'y': 北向距离(m), 'z': 天向距离(m)}
        """
        # 计算中点纬度（用于经度转换，减少误差）
        mid_lat = (lat + origin_lat) / 2.0
        
        # 使用 WGS84 椭球公式计算转换系数
        meters_per_lat = GeoUtils.meters_per_lat_degree(mid_lat)
        meters_per_lon = GeoUtils.meters_per_lon_degree(mid_lat)
        
        # 纬度差 → 北向距离 (Y轴)
        dlat = lat - origin_lat
        y = dlat * meters_per_lat
        
        # 经度差 → 东向距离 (X轴)
        dlon = lon - origin_lon
        x = dlon * meters_per_lon
        
        # 海拔差 → 天向距离 (Z轴)
        z = alt - origin_alt
        
        return {'x': x, 'y': y, 'z': z}
