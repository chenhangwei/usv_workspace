#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of param diagnostic.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
MAVROS 参数服务诊断脚本
用于检查 param 插件的状态和可用性
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import ParamPull, ParamGet
from mavros_msgs.msg import Param
import time


class ParamDiagnostic(Node):
    def __init__(self):
        super().__init__('param_diagnostic')
        
        namespace = 'usv_03'
        
        print(f"\n{'='*60}")
        print(f"MAVROS 参数服务诊断")
        print(f"命名空间: {namespace}")
        print(f"{'='*60}\n")
        
        # 1. 检查服务是否存在
        print("1. 检查服务可用性...")
        pull_service = f'/{namespace}/param/pull'
        get_service = f'/{namespace}/param/get'
        
        pull_client = self.create_client(ParamPull, pull_service)
        get_client = self.create_client(ParamGet, get_service)
        
        print(f"   ParamPull 服务: {pull_service}")
        if pull_client.wait_for_service(timeout_sec=3.0):
            print(f"   [OK] ParamPull 服务可用")
        else:
            print(f"   [X] ParamPull 服务不可用（超时 3秒）")
            return
        
        print(f"\n   ParamGet 服务: {get_service}")
        if get_client.wait_for_service(timeout_sec=3.0):
            print(f"   [OK] ParamGet 服务可用")
        else:
            print(f"   [X] ParamGet 服务不可用（超时 3秒）")
            return
        
        # 2. 测试 ParamGet（获取单个参数）
        print(f"\n2. 测试 ParamGet（获取单个参数 SYSID_THISMAV）...")
        request = ParamGet.Request()
        request.param_id = "SYSID_THISMAV"
        
        future = get_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    print(f"   [OK] ParamGet 成功")
                    print(f"   参数: {request.param_id}")
                    print(f"   值: {response.value.integer if response.value.integer != 0 else response.value.real}")
                else:
                    print(f"   [X] ParamGet 失败")
            except Exception as e:
                print(f"   [X] ParamGet 异常: {e}")
        else:
            print(f"   [X] ParamGet 超时（5秒）")
            print(f"   → 可能原因：飞控通信异常或参数同步未完成")
            return
        
        # 3. 订阅参数 topic
        print(f"\n3. 订阅参数 topic: /{namespace}/mavros/param/param_value")
        self.param_count = 0
        self.param_sub = self.create_subscription(
            Param,
            f'/{namespace}/mavros/param/param_value',
            self.param_callback,
            10
        )
        print(f"   [OK] 订阅成功，等待参数消息...")
        
        # 4. 调用 ParamPull
        print(f"\n4. 调用 ParamPull（拉取所有参数）...")
        pull_request = ParamPull.Request()
        pull_request.force_pull = False  # 不强制重新拉取
        
        print(f"   发送请求...")
        pull_future = pull_client.call_async(pull_request)
        
        start_time = time.time()
        timeout = 10.0
        
        while not pull_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                print(f"   [X] ParamPull 超时（{timeout}秒）")
                print(f"   → 可能原因：")
                print(f"      - 飞控未完全初始化")
                print(f"      - MAVROS param 插件配置问题")
                print(f"      - 飞控通信链路异常")
                return
        
        try:
            pull_response = pull_future.result()
            print(f"   [OK] ParamPull 响应")
            print(f"   成功: {pull_response.success}")
            print(f"   接收参数数: {pull_response.param_received}")
            
            if pull_response.success:
                # 等待参数通过 topic 发布
                print(f"\n5. 等待参数通过 topic 发布...")
                wait_time = 5.0
                start = time.time()
                
                while time.time() - start < wait_time:
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                print(f"   [OK] 通过 topic 接收到 {self.param_count} 个参数")
                
                if self.param_count == 0:
                    print(f"   [!]  警告：ParamPull 成功但未收到参数消息")
                    print(f"   → 可能原因：参数已在缓存中，未重新发布")
                elif self.param_count < pull_response.param_received:
                    print(f"   [!]  警告：接收参数数少于预期")
                    print(f"   预期: {pull_response.param_received}")
                    print(f"   实际: {self.param_count}")
            else:
                print(f"   [X] ParamPull 失败")
                
        except Exception as e:
            print(f"   [X] ParamPull 异常: {e}")
        
        print(f"\n{'='*60}")
        print(f"诊断完成")
        print(f"{'='*60}\n")
    
    def param_callback(self, msg):
        """参数 topic 回调"""
        self.param_count += 1
        if self.param_count <= 5 or self.param_count % 100 == 0:
            param_name = msg.header.frame_id
            value = msg.value.integer if msg.value.integer != 0 else msg.value.real
            print(f"   收到参数: {param_name} = {value}")


def main():
    rclpy.init()
    node = ParamDiagnostic()
    # 节点在初始化时已经完成所有测试
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
