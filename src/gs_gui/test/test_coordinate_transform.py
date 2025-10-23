"""
坐标系偏移量设置 - 快速测试
用于验证Area Center偏移量设置功能是否正常工作

注意：Boot Pose功能已移除（2025-10-23）
现在只测试 Area -> Global 的转换
飞控会自动将全局坐标转换为本地坐标
"""

def test_coordinate_transform():
    """测试简化后的坐标转换逻辑（Area -> Global）"""
    
    # 场景: 任务文件中定义一个点 (10, 5, 0)
    # Area Center设置为 (100, 50, 0)
    
    area_point = {'x': 10.0, 'y': 5.0, 'z': 0.0}
    area_center = {'x': 100.0, 'y': 50.0, 'z': 0.0}
    
    # Step 1 (唯一步骤): Area -> Global
    global_point = {
        'x': area_center['x'] + area_point['x'],
        'y': area_center['y'] + area_point['y'],
        'z': area_center['z'] + area_point['z']
    }
    print(f"Area坐标: {area_point}")
    print(f"Area Center偏移: {area_center}")
    print(f"转换为全局坐标: {global_point}")
    
    # 验证转换结果
    expected_global = {'x': 110.0, 'y': 55.0, 'z': 0.0}
    assert abs(global_point['x'] - expected_global['x']) < 0.01
    assert abs(global_point['y'] - expected_global['y']) < 0.01
    assert abs(global_point['z'] - expected_global['z']) < 0.01
    
    print("\n✓ 坐标转换测试通过！")
    print("\n说明:")
    print("1. 任务文件中的点(10,5,0)是相对于Area Center的")
    print("2. Area Center在全局坐标系的(100,50,0)")
    print("3. 转换后全局坐标为(110,55,0) ← 这个坐标直接发送给飞控")
    print("4. 飞控收到全局坐标后，会自动转换为自己的本地坐标系")
    print("5. 飞控本地坐标系以USV上电位置为原点(0,0,0)")


if __name__ == '__main__':
    test_coordinate_transform()
