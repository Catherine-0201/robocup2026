#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
语法测试脚本
"""

import sys
import os

def test_syntax():
    """测试Python文件语法"""
    script_path = "lidar_localization_monitor.py"
    
    if not os.path.exists(script_path):
        print(f"错误: 文件 {script_path} 不存在")
        return False
    
    try:
        with open(script_path, 'r', encoding='utf-8') as f:
            source = f.read()
        
        # 尝试编译
        compile(source, script_path, 'exec')
        print("✓ 语法检查通过")
        return True
        
    except SyntaxError as e:
        print(f"✗ 语法错误: {e}")
        print(f"  行号: {e.lineno}")
        print(f"  位置: {e.offset}")
        print(f"  错误: {e.text}")
        return False
    except Exception as e:
        print(f"✗ 其他错误: {e}")
        return False

if __name__ == "__main__":
    success = test_syntax()
    sys.exit(0 if success else 1)
