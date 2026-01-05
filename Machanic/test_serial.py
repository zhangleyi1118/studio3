"""
串口通信测试脚本 - 检查Arduino是否正确接收命令
"""
import serial
import serial.tools.list_ports
import time

def list_ports():
    ports = serial.tools.list_ports.comports()
    print("\n=== 可用串口 ===")
    for i, port in enumerate(ports):
        print(f"{i+1}. {port.device} - {port.description}")
    return ports

def main():
    ports = list_ports()
    if not ports:
        print("错误：没有找到串口！")
        return
    
    choice = int(input(f"\n选择串口 (1-{len(ports)}): ")) - 1
    selected_port = ports[choice].device
    
    try:
        ser = serial.Serial(selected_port, 115200, timeout=1)
        print(f"\n✓ 已连接到 {selected_port}")
        print("等待Arduino初始化...")
        time.sleep(2)
        
        # 读取初始化信息
        print("\n--- Arduino启动信息 ---")
        while ser.in_waiting:
            msg = ser.readline().decode('utf-8', errors='ignore').strip()
            if msg:
                print(f"  {msg}")
        
        print("\n--- 开始测试 ---")
        
        # 测试1：发送单字符 'a'
        print("\n[测试1] 发送: a")
        ser.write(b'a\n')
        time.sleep(0.3)
        
        response_found = False
        while ser.in_waiting:
            msg = ser.readline().decode('utf-8', errors='ignore').strip()
            if msg:
                print(f"  ← {msg}")
                response_found = True
        
        if not response_found:
            print("  ⚠️ 没有收到响应！")
        
        # 测试2：发送底层命令
        print("\n[测试2] 发送: A1,F,255")
        ser.write(b'A1,F,255\n')
        time.sleep(0.3)
        
        response_found = False
        while ser.in_waiting:
            msg = ser.readline().decode('utf-8', errors='ignore').strip()
            if msg:
                print(f"  ← {msg}")
                response_found = True
        
        if not response_found:
            print("  ⚠️ 没有收到响应！")
        
        # 测试3：停止
        print("\n[测试3] 发送: s")
        ser.write(b's\n')
        time.sleep(0.3)
        
        while ser.in_waiting:
            msg = ser.readline().decode('utf-8', errors='ignore').strip()
            if msg:
                print(f"  ← {msg}")
        
        print("\n--- 测试完成 ---")
        print("\n如果上面没有看到 'Cmd A:' 或 'Actuator' 开头的消息，")
        print("说明Arduino没有正确接收或处理命令！")
        
        ser.close()
        
    except Exception as e:
        print(f"错误: {e}")

if __name__ == "__main__":
    main()
