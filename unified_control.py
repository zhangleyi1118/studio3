"""
统一控制系统 - 同时控制Arduino和ESP32
保留现有控制脚本不变，新脚本直接连接两个设备并发送命令
"""

import serial
import serial.tools.list_ports
import time
import threading
import queue

class UnifiedController:
    """统一控制器，管理Arduino和ESP32两个设备"""
    
    def __init__(self):
        self.arduino_ser = None  # 机械系统（Arduino）
        self.esp32_ser = None    # LED系统（ESP32）
        self.running = True
        self.message_queue = queue.Queue()
    
    def list_available_ports(self):
        """列出所有可用的串口"""
        ports = serial.tools.list_ports.comports()
        print("\n=== 可用串口 ===")
        for i, port in enumerate(ports):
            print(f"{i+1}. {port.device} - {port.description}")
        return ports
    
    def connect_device(self, device_name, default_port=None):
        """连接到指定设备"""
        ports = self.list_available_ports()
        
        if not ports:
            print(f"错误：没有找到可用的串口！")
            return None
        
        print(f"\n【连接 {device_name}】")
        
        # 如果指定了默认端口，尝试使用
        if default_port:
            for port in ports:
                if port.device == default_port:
                    selected_port = default_port
                    print(f"使用指定端口: {selected_port}")
                    break
            else:
                print(f"警告：指定的端口 {default_port} 不存在，请手动选择")
                default_port = None
        
        # 如果没有默认端口或默认端口不存在，让用户选择
        if not default_port:
            while True:
                try:
                    choice = input(f"请选择串口 (1-{len(ports)}, 回车使用第一个): ").strip()
                    if not choice:
                        # 回车使用第一个
                        port_index = 0
                    else:
                        port_index = int(choice) - 1
                    
                    if 0 <= port_index < len(ports):
                        break
                    print("无效选择，请重试")
                except ValueError:
                    print("请输入数字或直接回车")
            
            selected_port = ports[port_index].device
        
        try:
            ser = serial.Serial(selected_port, 115200, timeout=1)
            print(f"✓ 已连接到 {device_name}: {selected_port}")
            print("等待设备初始化...")
            time.sleep(2)  # 等待设备重启
            
            # 读取初始化消息
            init_messages = []
            while ser.in_waiting:
                msg = ser.readline().decode('utf-8', errors='ignore').strip()
                if msg:
                    init_messages.append(msg)
                    print(f"  {device_name}: {msg}")
            
            return ser
        
        except serial.SerialException as e:
            print(f"错误：无法连接到 {selected_port}")
            print(f"详情: {e}")
            return None
    
    def send_command(self, ser, command, device_name):
        """向设备发送命令"""
        if ser is None or not ser.is_open:
            print(f"警告：{device_name} 未连接，跳过命令")
            return False
        
        try:
            ser.write((command + "\n").encode())
            print(f"  → {device_name}: {command}")
            time.sleep(0.1)
            return True
        except Exception as e:
            print(f"错误：向 {device_name} 发送命令失败 - {e}")
            return False
    
    def read_responses(self):
        """读取设备响应（非阻塞）"""
        # 读取Arduino响应
        if self.arduino_ser and self.arduino_ser.is_open:
            try:
                response_count = 0
                while self.arduino_ser.in_waiting and response_count < 10:
                    msg = self.arduino_ser.readline().decode('utf-8', errors='ignore').strip()
                    if msg:
                        print(f"  ← Arduino: {msg}")
                        response_count += 1
            except:
                pass
        
        # 读取ESP32响应
        if self.esp32_ser and self.esp32_ser.is_open:
            try:
                response_count = 0
                while self.esp32_ser.in_waiting and response_count < 10:
                    msg = self.esp32_ser.readline().decode('utf-8', errors='ignore').strip()
                    if msg:
                        # 检查是否是波生成信号
                        if msg.startswith("WAVE_SPAWN"):
                            parts = msg.split()
                            wave_info = {}
                            for part in parts[1:]:
                                if '=' in part:
                                    key, value = part.split('=')
                                    wave_info[key] = value
                            n_val = wave_info.get('n', '?')
                            speed_val = wave_info.get('speed', '?')
                            phase_val = wave_info.get('phase', '?')
                            print(f"  ← ESP32: 🌊 [波生成] n={n_val}, 速度={speed_val}, 相位={phase_val}")
                        else:
                            print(f"  ← ESP32: {msg}")
                        response_count += 1
            except:
                pass
    
    def parse_command(self, command):
        """
        解析命令并决定发送到哪些设备
        返回: (arduino_cmd, esp32_cmd, mode)
        mode: 'both'=联动, 'arduino'=仅Arduino, 'esp32'=仅ESP32, 'help'=帮助, 'unknown'=未知
        """
        command_lower = command.lower().strip()
        
        # 退出命令
        if command_lower in ['q', 'quit']:
            return ('s', 'q', 'both')  # Arduino停止，ESP32关闭灯
        
        # 帮助命令
        if command_lower in ['h', 'help']:
            return (None, None, 'help')
        
        # 联动命令：f,<值> - 同时控制亮度和前进
        if command_lower.startswith('f,'):
            try:
                parts = command_lower.split(',')
                if len(parts) == 2:
                    value = float(parts[1])
                    if 0 <= value <= 100:
                        # 联动：LED设置亮度，机械前进相同百分比
                        return (f'f,{value}', f'f,{value}', 'both')
                    else:
                        return (None, None, 'error')
            except (ValueError, IndexError):
                return (None, None, 'error')
        
        # 联动命令：b,<值> - 机械后退，LED亮度降低
        if command_lower.startswith('b,'):
            try:
                parts = command_lower.split(',')
                if len(parts) == 2:
                    value = float(parts[1])
                    if 0 <= value <= 100:
                        # 联动：机械后退，LED亮度降低（映射到0-100）
                        led_value = max(0, 100 - value)  # 后退时亮度降低
                        return (f'b,{value}', f'f,{led_value}', 'both')
                    else:
                        return (None, None, 'error')
            except (ValueError, IndexError):
                return (None, None, 'error')
        
        # 暂停/停止命令
        if command_lower == 's':
            return ('s', 's', 'both')  # 两个设备都暂停/停止
        
        # START命令 - 仅Arduino
        if command_lower.startswith('start'):
            return (command, None, 'arduino')
        
        # 调试模式命令 - 仅Arduino
        if any(keyword in command_lower for keyword in ['group1', 'group2', 'stepper', 'servo']):
            return (command, None, 'arduino')
        
        # 默认：如果无法识别，尝试发送到两个设备（可能是不认识的命令）
        # 或者返回错误
        return (None, None, 'unknown')
    
    def print_help(self):
        """打印帮助信息"""
        print("\n" + "=" * 70)
        print("统一控制系统 - 指令说明")
        print("=" * 70)
        
        print("\n【联动模式】（同时控制Arduino和ESP32）")
        print("  f,<0-100>  - 联动命令：")
        print("                • ESP32: 设置LED亮度为 <值>")
        print("                • Arduino: 前进 <值>%")
        print("                示例: f,50  → LED亮度50，机械前进50%")
        
        print("\n  b,<0-100>  - 联动命令：")
        print("                • Arduino: 后退 <值>%")
        print("                • ESP32: LED亮度降低（100-<值>）")
        print("                示例: b,30  → 机械后退30%，LED亮度70")
        
        print("\n  s          - 联动停止：")
        print("                • Arduino: 强制停止所有运动")
        print("                • ESP32: 暂停/恢复虚拟位置移动")
        
        print("\n【Arduino独立控制】")
        print("  START[...]  - 恢复初始状态")
        print("  GROUP1/2,<方向>,<百分比>  - 控制伸缩杆组")
        print("  STEPPER,<方向>,<百分比>   - 控制步进电机")
        print("  SERVO,<角度>,<百分比>      - 控制舵机")
        print("  说明：这些命令只发送到Arduino，不影响ESP32")
        
        print("\n【系统命令】")
        print("  q  - 退出程序（Arduino停止，ESP32关闭所有灯）")
        print("  h  - 显示此帮助信息")
        print("=" * 70)
        
        print("\n【提示】")
        print("  • 现有脚本 control_led.py 和 control_test.py 保持不变")
        print("  • 可以继续使用原有脚本单独控制设备")
        print("  • 本脚本用于统一联动控制两个设备")
        print("=" * 70)
    
    def run(self):
        """主运行循环"""
        print("=" * 70)
        print("统一控制系统 - Arduino + ESP32")
        print("=" * 70)
        print("本脚本直接连接两个设备，不依赖现有控制脚本")
        print("=" * 70)
        
        # 连接Arduino
        self.arduino_ser = self.connect_device("Arduino（机械系统）")
        if self.arduino_ser is None:
            print("警告：Arduino未连接，将无法控制机械系统")
        
        # 连接ESP32
        self.esp32_ser = self.connect_device("ESP32（LED系统）")
        if self.esp32_ser is None:
            print("警告：ESP32未连接，将无法控制LED系统")
        
        if self.arduino_ser is None and self.esp32_ser is None:
            print("\n错误：没有设备连接成功，程序退出")
            return
        
        self.print_help()
        
        # 启动响应读取线程
        def read_loop():
            while self.running:
                self.read_responses()
                time.sleep(0.1)
        
        read_thread = threading.Thread(target=read_loop, daemon=True)
        read_thread.start()
        
        try:
            while True:
                # 获取用户输入
                try:
                    command = input("\n输入指令: ").strip()
                except (EOFError, KeyboardInterrupt):
                    break
                
                if not command:
                    continue
                
                # 解析命令
                arduino_cmd, esp32_cmd, mode = self.parse_command(command)
                
                if mode == 'help':
                    self.print_help()
                    continue
                
                if mode == 'error':
                    print("错误：命令格式不正确")
                    print("提示：使用 'h' 查看帮助信息")
                    continue
                
                if mode == 'unknown':
                    print("错误：无法识别的命令格式")
                    print("提示：使用 'h' 查看帮助信息")
                    continue
                
                # 发送命令
                print(f"\n【执行命令: {command}】")
                
                if mode == 'both':
                    # 联动模式：同时发送到两个设备
                    if arduino_cmd:
                        self.send_command(self.arduino_ser, arduino_cmd, "Arduino")
                    if esp32_cmd:
                        self.send_command(self.esp32_ser, esp32_cmd, "ESP32")
                
                elif mode == 'arduino':
                    # 仅Arduino
                    if arduino_cmd:
                        self.send_command(self.arduino_ser, arduino_cmd, "Arduino")
                    else:
                        print("警告：Arduino命令为空")
                
                elif mode == 'esp32':
                    # 仅ESP32
                    if esp32_cmd:
                        self.send_command(self.esp32_ser, esp32_cmd, "ESP32")
                    else:
                        print("警告：ESP32命令为空")
                
                time.sleep(0.2)  # 等待设备响应
                
                # 读取响应
                self.read_responses()
        
        except KeyboardInterrupt:
            print("\n\n检测到 Ctrl+C，正在退出...")
        
        finally:
            self.running = False
            
            # 退出前发送停止命令
            print("\n正在关闭设备...")
            if self.arduino_ser:
                try:
                    self.send_command(self.arduino_ser, 's', "Arduino")
                except:
                    pass
            
            if self.esp32_ser:
                try:
                    self.send_command(self.esp32_ser, 'q', "ESP32")
                except:
                    pass
            
            time.sleep(0.5)
            
            # 关闭串口
            if self.arduino_ser and self.arduino_ser.is_open:
                self.arduino_ser.close()
                print("✓ Arduino串口已关闭")
            
            if self.esp32_ser and self.esp32_ser.is_open:
                self.esp32_ser.close()
                print("✓ ESP32串口已关闭")
            
            print("程序已退出")

def main():
    controller = UnifiedController()
    controller.run()

if __name__ == "__main__":
    main()
