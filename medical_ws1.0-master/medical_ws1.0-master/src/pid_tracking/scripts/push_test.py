#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import time

class PushTest:
    def __init__(self):
        """初始化串口连接"""
        self.arm_serial = self.init_arm_serial()
        
    def init_arm_serial(self):
        """初始化机械臂串口连接"""
        port = '/dev/arm'
        baud_rate = 115200
        try:
            ser = serial.Serial(port, baud_rate, timeout=1)
            if ser.is_open:
                print(f'串口 {port} 成功打开，波特率: {baud_rate}')
                return ser
            else:
                print(f'无法打开串口 {port}')
                return None
        except serial.SerialException as e:
            print(f'打开串口时发生错误: {e}')
            return None
    
    def send_command_and_wait_response(self, task):
        """
        发送指令并等待回复
        
        参数:
            task (int): 任务编号，1或3
            
        返回:
            tuple: (success, response_data)
                success (bool): 是否成功
                response_data (str): 返回的数据
        """
        if self.arm_serial is None:
            print("串口未初始化成功")
            return False, "串口错误"
        
        # 根据task值确定发送的指令
        if task == 1:
            command = '1\r'
            print("准备发送指令: '1\\r' (执行任务1)")
        elif task == 3:
            command = '3\r'
            print("准备发送指令: '3\\r' (执行任务3)")
        else:
            print(f"无效的任务编号: {task}，只支持1或3")
            return False, "无效任务编号"
        
        try:
            # 步骤1: 发送指令给上位机
            print("正在发送指令...")
            self.arm_serial.write(command.encode('utf-8'))
            print(f"指令已发送: {repr(command)}")
            
            # 步骤2: 等待上位机返回确认数据
            print("等待上位机回复...")
            timeout_counter = 0
            max_timeout = 100  # 最大等待次数，避免无限等待
            
            while not self.arm_serial.in_waiting and timeout_counter < max_timeout:
                time.sleep(0.1)  # 短暂等待
                timeout_counter += 1
            
            if timeout_counter >= max_timeout:
                print("等待回复超时")
                return False, "超时"
            
            # 步骤3: 读取上位机返回的消息
            data = self.arm_serial.readline().decode('utf-8').strip()
            print(f"收到回复: '{data}'")
            
            # 步骤4: 处理返回结果
            if data:
                print(f"指令执行成功，收到数据: {data}")
                print("等待2秒让动作完成...")
                time.sleep(2)  # 等待2秒让动作完成
                return True, data
            else:
                print("没有收到有效数据")
                return False, "无数据"
                
        except Exception as e:
            print(f"发送指令或接收数据时发生错误: {e}")
            return False, str(e)
    
    def close_serial(self):
        """关闭串口连接"""
        if self.arm_serial and self.arm_serial.is_open:
            self.arm_serial.close()
            print("串口连接已关闭")

def main():
    """主函数 - 测试用例"""
    print("=== 机械臂推药测试程序 ===")
    
    # 创建测试实例
    push_test = PushTest()
    
    try:
        # 获取用户输入
        print("\n请输入任务编号:")
        print("1 - 执行任务1（发送'1\\r'）")
        print("3 - 执行任务3（发送'3\\r'）")
        
        task = int(input("请输入任务编号 (1 或 3): "))
        
        if task not in [1, 3]:
            print("错误：只支持任务编号 1 或 3")
            return
        
        print(f"\n开始执行任务 {task}...")
        
        # 发送指令并等待回复
        success, response = push_test.send_command_and_wait_response(task)
        
        # 输出结果
        print("\n=== 执行结果 ===")
        if success:
            print(f"✓ 任务执行成功")
            print(f"✓ 收到回复: {response}")
        else:
            print(f"✗ 任务执行失败")
            print(f"✗ 错误信息: {response}")
    
    except ValueError:
        print("错误：请输入有效的数字")
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"程序执行出错: {e}")
    
    finally:
        # 清理资源
        push_test.close_serial()

if __name__ == '__main__':
    main()

