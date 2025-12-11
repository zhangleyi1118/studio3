"""
Arduino 伸缩杆控制测试脚本
通过键盘输入 f/b/s 控制伸缩杆
"""

import serial
import serial.tools.list_ports
import time

def list_available_ports():
    """列出所有可用的串口"""
    ports = serial.tools.list_ports.comports()
    print("\n=== 可用串口 ===")
    for i, port in enumerate(ports):
        print(f"{i+1}. {port.device} - {port.description}")
    return ports

def connect_arduino():
    """连接到Arduino"""
    ports = list_available_ports()
    
    if not ports:
        print("错误：没有找到可用的串口！")
        return None
    
    # 让用户选择串口
    while True:
        try:
            choice = input(f"\n请选择串口 (1-{len(ports)}): ")
            port_index = int(choice) - 1
            if 0 <= port_index < len(ports):
                break
            print("无效选择，请重试")
        except ValueError:
            print("请输入数字")
    
    selected_port = ports[port_index].device
    
    try:
        # 连接Arduino (波特率 115200)
        ser = serial.Serial(selected_port, 115200, timeout=1)
        print(f"\n✓ 已连接到 {selected_port}")
        print("等待Arduino初始化...")
        time.sleep(2)  # 等待Arduino重启
        
        # 读取Arduino的初始化消息
        while ser.in_waiting:
            msg = ser.readline().decode('utf-8', errors='ignore').strip()
            if msg:
                print(f"Arduino: {msg}")
        
        return ser
    
    except serial.SerialException as e:
        print(f"错误：无法连接到 {selected_port}")
        print(f"详情: {e}")
        return None

def main():
    print("=" * 50)
    print("Arduino 伸缩杆控制程序")
    print("=" * 50)
    
    # 连接Arduino
    ser = connect_arduino()
    if ser is None:
        return
    
    print("\n" + "=" * 50)
    print("控制指令:")
    print("  a = 高层命令A：1-2前伸200；3-6后缩100；7-14停")
    print("  b = 高层命令B：1-2后缩200；3-6前伸100；7-14停")
    print("  s = 全部停止")
    print("  q = 退出程序 (Quit)")
    print("=" * 50)
    
    try:
        while True:
            # 读取Arduino的反馈消息
            while ser.in_waiting:
                msg = ser.readline().decode('utf-8', errors='ignore').strip()
                if msg:
                    print(f"Arduino: {msg}")
            
            # 获取用户输入
            command = input("\n输入指令 (a/b/s/q): ").strip().lower()
            
            if command == 'q':
                print("\n正在退出...")
                ser.write(b's')  # 退出前先停止电机
                time.sleep(0.1)
                break
            
            elif command in ['a', 'b', 's']:
                # 发送高层指令到Arduino（加换行便于解析）
                ser.write((command + "\n").encode())
                print(f"✓ 已发送指令: {command.upper()}")
                time.sleep(0.2)  # 等待Arduino响应
                
                # 立即读取Arduino返回
                response_count = 0
                while ser.in_waiting and response_count < 10:
                    msg = ser.readline().decode('utf-8', errors='ignore').strip()
                    if msg:
                        print(f"  ← Arduino: {msg}")
                        response_count += 1
                
            else:
                print("⚠ 无效指令，请输入 f/b/s/q")
    
    except KeyboardInterrupt:
        print("\n\n检测到 Ctrl+C，正在退出...")
        ser.write(b's\n')  # 确保停止电机
        time.sleep(0.1)
    
    finally:
        if ser and ser.is_open:
            ser.close()
            print("✓ 串口已关闭")
        print("程序已退出")

if __name__ == "__main__":
    main()
