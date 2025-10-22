# ROS2 测试使用指南

## 测试基础概念

### 1. 测试类型

#### 代码质量测试（Linting Tests）
- **目的**: 检查代码格式、风格、文档等
- **文件**: `test_flake8.py`, `test_pep257.py`, `test_copyright.py`
- **何时运行**: 每次提交代码前

#### 单元测试（Unit Tests）  
- **目的**: 测试单个函数、类的功能
- **文件**: `test_*.py`（如 `test_demo.py`）
- **何时运行**: 开发新功能时

#### 集成测试（Integration Tests）
- **目的**: 测试多个组件协同工作
- **适用场景**: 测试ROS节点间通信

## 2. 运行测试的方法

### 运行所有测试
```bash
# 构建并运行所有测试
colcon test

# 查看测试结果
colcon test-result --verbose
```

### 运行特定包的测试
```bash
# 只运行gs_gui包的测试
colcon test --packages-select gs_gui

# 运行测试并显示详细输出
colcon test --packages-select gs_gui --pytest-args "-v"
```

### 运行特定测试文件
```bash
cd src/gs_gui
python3 -m pytest test/test_demo.py -v
```

### 运行特定测试方法
```bash
python3 -m pytest test/test_demo.py::TestROSSignal::test_simple_math -v
```

## 3. 代码质量检查

### 手动检查代码风格
```bash
# 检查单个文件
python3 -m flake8 gs_gui/main_gui_app.py

# 检查整个包
python3 -m flake8 gs_gui/
```

### 自动修复格式问题
```bash
# 显示会修复的问题（不实际修改）
python3 -m autopep8 --diff gs_gui/main_gui_app.py

# 实际修复文件
python3 -m autopep8 --in-place gs_gui/main_gui_app.py
```

## 4. 编写测试的最佳实践

### 测试文件结构
```python
import unittest
from unittest.mock import Mock, patch

class TestMyClass(unittest.TestCase):
    def setUp(self):
        """每个测试前运行"""
        pass
    
    def tearDown(self):
        """每个测试后运行"""  
        pass
    
    def test_specific_function(self):
        """测试特定功能"""
        # 准备测试数据
        expected = "expected_result"
        
        # 执行要测试的代码
        actual = my_function()
        
        # 验证结果
        self.assertEqual(actual, expected)
```

### ROS节点测试模式
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestRosNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod  
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def test_node_creation(self):
        node = Node('test_node')
        self.assertIsNotNone(node)
        node.destroy_node()
```

## 5. 常见测试场景

### 测试消息发布
```python
def test_message_publishing(self):
    # 创建测试节点
    node = TestPublisher()
    
    # 创建订阅者收集消息
    received_messages = []
    def callback(msg):
        received_messages.append(msg.data)
    
    subscription = node.create_subscription(
        String, 'test_topic', callback, 10)
    
    # 发布测试消息
    node.publish_test_message("test_data")
    
    # 等待消息处理
    rclpy.spin_once(node, timeout_sec=1.0)
    
    # 验证消息
    self.assertEqual(len(received_messages), 1)
    self.assertEqual(received_messages[0], "test_data")
```

### 测试Action服务器
```python
def test_action_server(self):
    # 使用Mock模拟Action客户端
    mock_client = Mock()
    mock_goal = Mock()
    mock_goal.goal.target_position = [1.0, 2.0, 0.0]
    
    # 测试目标处理
    result = action_server.execute_callback(mock_goal)
    
    # 验证结果
    self.assertTrue(result.success)
```

## 6. 跳过和条件测试

### 跳过特定测试
```python
@unittest.skip("功能尚未实现")
def test_future_feature(self):
    pass

@unittest.skipIf(condition, "跳过原因")  
def test_conditional(self):
    pass
```

### 根据硬件条件跳过
```python
@unittest.skipIf(not os.path.exists('/dev/ttyUSB0'), "无串口设备")
def test_serial_communication(self):
    pass
```

## 7. 调试失败的测试

### 增加调试输出
```python
def test_debug_example(self):
    result = complex_function()
    print(f"调试输出: {result}")  # 临时调试
    self.assertEqual(result, expected_value)
```

### 使用pdb调试器
```python
def test_with_debugger(self):
    import pdb; pdb.set_trace()  # 设置断点
    result = my_function()
    self.assertTrue(result)
```

## 8. 测试配置文件

修改 `setup.cfg` 来配置测试行为：

```ini
[tool:pytest]
testpaths = test
addopts = -v --tb=short

[flake8]  
max-line-length = 120
ignore = E203,W503,D100
```

## 总结

- **经常运行测试**: 每次修改代码后都运行相关测试
- **先写测试**: 采用TDD（测试驱动开发）方式
- **保持简单**: 每个测试只验证一个功能点
- **使用Mock**: 对外部依赖（硬件、网络）使用Mock
- **清晰命名**: 测试方法名要清楚表达测试内容