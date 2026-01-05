"""
Arduino 多组件同步控制系统 - Python控制脚本
支持主工作模式和调试模式
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

def print_help():
    """打印帮助信息"""
    print("\n" + "=" * 60)
    print("控制指令说明")
    print("=" * 60)
    print("\n【主工作模式】（执行→停止10s→自动复位）")
    print("  f,<百分比>  - 前进（例如: f,20 表示前进20%）")
    print("  b,<百分比>  - 后退（例如: b,50 表示后退50%）")
    print("  说明：执行后停止10秒，然后自动反向复位相同时间")
    print("  注意：每次指令后有30s控制间隙（除强制停止外）")
    
    print("\n【调试模式】（单独控制组件）")
    print("  GROUP1,<方向>,<百分比>  - 控制组1伸缩杆（1-6）")
    print("  GROUP2,<方向>,<百分比>  - 控制组2伸缩杆（7-14）")
    print("  STEPPER,<方向>,<百分比> - 控制步进电机滑台")
    print("  SERVO,<角度>,<百分比>   - 控制舵机")
    print("  方向：F=前进/右移, B=后退/左移, L=左移, R=右移, S=停止")
    print("  示例：")
    print("    GROUP1,F,20      - 组1前进20%")
    print("    STEPPER,L,30     - 步进电机左移30%")
    print("    SERVO,180,50     - 舵机转到180度的50%（即90度）")
    print("    GROUP1+STEPPER,F,20  - 组合控制：组1和步进电机同时前进20%")
    
    print("\n【初始状态恢复】")
    print("  START 或 START,ALL  - 恢复所有组件到初始状态")
    print("  START,ACTUATORS     - 只恢复伸缩杆（前进20cm，后退10cm，居中）")
    print("  START,STEPPER       - 只重置步进电机位置计数器为0（不实际移动）")
    print("  START,SERVO         - 只恢复舵机到0度")
    print("  START,ACTUATORS+SERVO - 组合恢复（例如）")
    
    print("\n【强制停止】")
    print("  s  - 强制停止所有运动，并清零控制间隙")
    
    print("\n【其他】")
    print("  h  - 显示此帮助信息")
    print("  q  - 退出程序")
    print("=" * 60)

def main():
    print("=" * 60)
    print("Arduino 多组件同步控制系统")
    print("=" * 60)
    
    # 连接Arduino
    ser = connect_arduino()
    if ser is None:
        return
    
    print_help()
    
    try:
        while True:
            # 读取Arduino的反馈消息
            while ser.in_waiting:
                msg = ser.readline().decode('utf-8', errors='ignore').strip()
                if msg:
                    print(f"Arduino: {msg}")
            
            # 获取用户输入
            try:
                command = input("\n输入指令: ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            
            if not command:
                continue
            
            command_lower = command.lower()
            
            # 退出命令
            if command_lower == 'q' or command_lower == 'quit':
                print("\n正在退出...")
                ser.write(b's\n')  # 退出前先停止电机
                time.sleep(0.1)
                break
            
            # 帮助命令
            if command_lower == 'h' or command_lower == 'help':
                print_help()
                continue
            
            # 发送命令到Arduino（加换行便于解析）
            try:
                ser.write((command + "\n").encode())
                print(f"✓ 已发送指令: {command}")
                time.sleep(0.2)  # 等待Arduino响应
                
                # 立即读取Arduino返回
                response_count = 0
                while ser.in_waiting and response_count < 10:
                    msg = ser.readline().decode('utf-8', errors='ignore').strip()
                    if msg:
                        print(f"  ← {msg}")
                        response_count += 1
            except Exception as e:
                print(f"错误：发送命令失败 - {e}")
    
    except KeyboardInterrupt:
        print("\n\n检测到 Ctrl+C，正在退出...")
        try:
            ser.write(b's\n')  # 确保停止电机
            time.sleep(0.1)
        except:
            pass
    
    finally:
        if ser and ser.is_open:
            ser.close()
            print("✓ 串口已关闭")
        print("程序已退出")

if __name__ == "__main__":
    main()
