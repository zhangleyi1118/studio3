"""
LED灯带控制系统 - Python控制脚本
控制14根灯带的功能A（间隔点亮）和功能B（虚拟位置传播）
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
    """连接到Arduino/ESP32"""
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
        # 连接Arduino/ESP32 (波特率 115200)
        ser = serial.Serial(selected_port, 115200, timeout=1)
        print(f"\n✓ 已连接到 {selected_port}")
        print("等待设备初始化...")
        time.sleep(2)  # 等待设备重启
        
        # 读取设备的初始化消息
        while ser.in_waiting:
            msg = ser.readline().decode('utf-8', errors='ignore').strip()
            if msg:
                print(f"设备: {msg}")
        
        return ser
    
    except serial.SerialException as e:
        print(f"错误：无法连接到 {selected_port}")
        print(f"详情: {e}")
        return None

def print_help():
    """打印帮助信息"""
    print("\n" + "=" * 60)
    print("LED灯带控制指令说明")
    print("=" * 60)
    print("\n【亮度控制】")
    print("  f,<0-100>  - 设置亮度值（0最低，100最高）")
    print("              功能A：亮度映射到20-40范围")
    print("              功能B：亮度映射到0-100范围")
    print("              亮度从当前值平滑过渡到目标值，0→100需要16秒")
    print("              示例：")
    print("                f,0    - 设置亮度为0（最低）")
    print("                f,50   - 设置亮度为50（中等）")
    print("                f,100  - 设置亮度为100（最高）")
    
    print("\n【速度控制】")
    print("  功能B的虚拟位置传播速度：v = n + 10")
    print("  其中 n 是 f,n 命令中的亮度值")
    print("  例如：f,50 时，速度 v = 50 + 10 = 60 虚拟位置/秒")
    
    print("\n【暂停控制】")
    print("  s  - 暂停/恢复虚拟位置的移动")
    print("       暂停时保持当前亮度，恢复后继续过渡")
    
    print("\n【退出】")
    print("  q  - 退出程序并关闭所有灯")
    
    print("\n【其他】")
    print("  h  - 显示此帮助信息")
    print("=" * 60)
    
    print("\n【功能说明】")
    print("  功能A：灯带两端区域（0-25% 和 75-100%）")
    print("        - 间隔点亮：每隔2颗亮1颗")
    print("        - 亮度范围：20-40")
    print("  功能B：灯带中间区域（25-75%）")
    print("        - 虚拟位置1-20循环传播")
    print("        - 拖尾效果：前后各2个虚拟位置")
    print("        - 亮度范围：0-100")
    print("=" * 60)

def main():
    print("=" * 60)
    print("LED灯带控制系统")
    print("=" * 60)
    
    # 连接Arduino/ESP32
    ser = connect_arduino()
    if ser is None:
        return
    
    print_help()
    
    try:
        while True:
            # 读取设备的反馈消息
            while ser.in_waiting:
                msg = ser.readline().decode('utf-8', errors='ignore').strip()
                if msg:
                    print(f"设备: {msg}")
            
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
                ser.write(b'q\n')  # 发送退出命令，关闭所有灯
                time.sleep(0.1)
                break
            
            # 帮助命令
            if command_lower == 'h' or command_lower == 'help':
                print_help()
                continue
            
            # 验证命令格式
            if command_lower.startswith('f,'):
                # 验证f命令格式
                try:
                    parts = command_lower.split(',')
                    if len(parts) != 2:
                        print("错误：f命令格式应为 f,<0-100>")
                        continue
                    value = float(parts[1])
                    if value < 0 or value > 100:
                        print("错误：亮度值必须在0-100之间")
                        continue
                except ValueError:
                    print("错误：亮度值必须是数字")
                    continue
            
            # 发送命令到设备（加换行便于解析）
            try:
                ser.write((command + "\n").encode())
                print(f"✓ 已发送指令: {command}")
                time.sleep(0.2)  # 等待设备响应
                
                # 立即读取设备返回
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
            ser.write(b'q\n')  # 确保关闭所有灯
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
