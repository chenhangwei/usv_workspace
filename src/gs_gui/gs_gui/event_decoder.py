import json
import os


class EventDecoder:
    """
    PX4 事件接口解码器
    负责加载 events.json 元数据文件，并将事件 ID 解析为人类可读的文本
    """
    def __init__(self, logger):
        self.logger = logger
        self.events = {}
        self.enums = {}
        self._loaded = False
        
        # 尝试加载默认路径的元数据文件
        # 用户需要将 px4_events.json 放入 gs_gui/config/ 或指定目录
        self.default_path = os.path.expanduser('~/usv_workspace/src/gs_gui/config/px4_events.json')
        self.load_metadata(self.default_path)

    def load_metadata(self, path):
        """加载 PX4 事件元数据 JSON 文件"""
        try:
            if not os.path.exists(path):
                self.logger.warn(f"PX4事件元数据文件未找到: {path}")
                self.logger.warn("请从 QGC 或 PX4 源码下载 events.json 并重命名为 px4_events.json 放置于该路径")
                return

            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                
            # 解析 JSON 结构 (根据 PX4 events.json 格式)
            # 通常结构是: {"version": 1, "components": { "1": { "namespace": "px4", "events": { "12345": { "name": "...", "message": "..." } } } }}
            # 这里做一个简化的扁平化处理，建立 ID -> Event 的映射
            
            count = 0
            enum_count = 0
            
            # 处理新版 JSON 结构
            if 'components' in data:
                for comp_id, comp_data in data['components'].items():
                    # 加载枚举定义 (如果有)
                    if 'enums' in comp_data:
                        for enum_name, enum_def in comp_data['enums'].items():
                            self.enums[enum_name] = enum_def
                            enum_count += 1
                    
                    # 1. 直接在 component 下的 events (如果有)
                    if 'events' in comp_data:
                        for event_id, event_def in comp_data['events'].items():
                            try:
                                eid = int(event_id)
                                self.events[eid] = event_def
                                count += 1
                            except ValueError:
                                pass
                    
                    # 2. 在 event_groups 下的 events
                    if 'event_groups' in comp_data:
                        for group_name, group_data in comp_data['event_groups'].items():
                            if 'events' in group_data:
                                for event_id, event_def in group_data['events'].items():
                                    try:
                                        eid = int(event_id)
                                        self.events[eid] = event_def
                                        count += 1
                                    except ValueError:
                                        pass

            # 处理旧版或扁平结构 (如果有)
            elif 'events' in data:
                for event_id, event_def in data['events'].items():
                    eid = int(event_id)
                    self.events[eid] = event_def
                    count += 1
                    
            self._loaded = True
            self.logger.info(f"成功加载 PX4 事件定义: {count} 条, 枚举类型: {enum_count} 个")
            
        except Exception as e:
            self.logger.error(f"加载 PX4 事件元数据失败: {e}")

    def decode(self, event_id, args_str=None, args_bytes=None):
        """
        解码事件 ID
        
        PX4 事件 ID 结构: (component_id << 24) | sub_id
        px4_events.json 中使用的是 sub_id（24位）
        
        Args:
            event_id (int): 事件 ID（完整的32位 ID）
            args_str (str): 参数字符串 (例如 "-4-0-0...")
            args_bytes (bytes): 原始参数字节数组（优先使用）
            
        Returns:
            str: 解码后的消息，如果未找到定义则返回 None
        """
        if not self._loaded:
            return None
        
        # 提取 sub_id（低24位）用于查找事件定义
        sub_id = event_id & 0xFFFFFF
        
        # 先尝试用 sub_id 查找
        event = self.events.get(sub_id)
        
        # 如果没找到，尝试用完整的 event_id 查找（兼容旧格式）
        if not event:
            event = self.events.get(event_id)
        
        if event:
            # 获取消息模板
            message = event.get('message', '')
            name = event.get('name', f'Event {event_id}')
            
            # 如果 message 为空，尝试使用 name
            if not message:
                return f"[{name}]"
            
            # 尝试解析参数并填充占位符
            if args_str and '{' in message:
                message = self._fill_arguments(message, args_str, event)
            
            return message
            
        return None

    def _fill_arguments(self, message, args_str, event):
        """
        尝试填充消息模板中的参数占位符
        
        Args:
            message: 消息模板 (例如 "Armed by {1}")
            args_str: 参数字符串 (例如 "-4-0-0-0")
            event: 事件定义，包含参数类型信息
            
        Returns:
            str: 填充后的消息
        """
        import re
        
        try:
            # 解析参数字符串 (格式通常是 "-val1-val2-val3..." 或十六进制)
            # 简单处理：提取所有数字
            args = []
            
            # 处理不同格式的参数字符串
            if args_str.startswith('-'):
                # 格式: "-4-0-0-0"
                parts = args_str.split('-')
                for part in parts:
                    if part:
                        try:
                            args.append(int(part))
                        except ValueError:
                            args.append(part)
            else:
                # 尝试作为十六进制或其他格式
                args.append(args_str)
            
            # 获取事件定义中的参数信息 (如果有)
            event_args = event.get('arguments', [])
            
            # 替换占位符 {1}, {2}, ... (注意: PX4 使用 1-based 索引)
            def replace_placeholder(match):
                idx = int(match.group(1)) - 1  # 转换为 0-based
                if 0 <= idx < len(args):
                    value = args[idx]
                    # 如果有参数定义，尝试解析枚举值
                    if idx < len(event_args):
                        arg_def = event_args[idx]
                        if 'enum' in arg_def:
                            # 尝试查找枚举值的名称
                            enum_name = arg_def.get('enum')
                            enum_val = self._get_enum_value(enum_name, value)
                            if enum_val:
                                return enum_val
                    return str(value)
                return match.group(0)  # 保留原占位符
            
            # 替换 {1}, {2}, ... 格式的占位符
            result = re.sub(r'\{(\d+)\}', replace_placeholder, message)
            
            # 同时处理 {1:.0m}, {1:.1f} 等带格式的占位符
            result = re.sub(r'\{(\d+):[^}]+\}', replace_placeholder, result)
            
            return result
            
        except Exception as e:
            # 解析失败，返回原消息
            return message

    def _get_enum_value(self, enum_name, value):
        """
        获取枚举值的名称
        
        Args:
            enum_name: 枚举类型名称
            value: 枚举值
            
        Returns:
            str: 枚举值名称，如果未找到返回 None
        """
        if not self.enums:
            return None
        
        enum_def = self.enums.get(enum_name, {})
        entries = enum_def.get('entries', {})
        
        for entry_val, entry_def in entries.items():
            if int(entry_val) == value:
                return entry_def.get('name', str(value))
        
        return None
