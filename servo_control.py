import sys
import serial
from PySide6 import QtWidgets, QtCore
from PySide6.QtWidgets import QApplication, QMainWindow, QLabel, QSlider, QPushButton, QVBoxLayout, QWidget, QHBoxLayout
from PySide6.QtCore import Qt
import crcmod

# ================== CRC8 校验函数 ==================
crc8_func = crcmod.mkCrcFun(poly=0x107, initCrc=0x00, rev=False, xorOut=0x00)

# ================== 舵机类 ==================
class Servo:
    def __init__(self, servo_id, init_angle=90, speed=20, step=1, angle_min=0, angle_max=270):
        self.servo_id = servo_id
        self.angle = init_angle
        self.init_angle = init_angle
        self.speed = speed
        self.step = step
        self.angle_min = angle_min
        self.angle_max = angle_max

    def set_angle(self, new_angle):
        self.angle = max(self.angle_min, min(self.angle_max, new_angle))
        return self.angle

    def increase_angle(self):
        return self.set_angle(self.angle + self.step)

    def decrease_angle(self):
        return self.set_angle(self.angle - self.step)

    def build_command(self):
        angle_val = int(self.angle)
        low = (angle_val >> 8) & 0xFF
        high = angle_val & 0xFF
        cmd = [0xAA, self.servo_id, high, low, self.speed]
        crc = crc8_func(bytes(cmd))
        cmd.append(crc)
        return cmd

# ================== 控制对象类（水泵/照明灯/DVL） ==================
class Device:
    def __init__(self, dev_id, value=0):
        self.dev_id = dev_id
        self.value = value

    def set_value(self, val):
        self.value = max(0, min(99, val))
        return self.value

    def build_command(self):
        cmd = [0xAA, self.dev_id, self.value, 0x00, 0x00]
        crc = crc8_func(bytes(cmd))
        cmd.append(crc)
        return cmd

def initialize_devices(ser, servos, peripherals):
    # 初始化舵机到初始角度
    for servo in servos.values():
        cmd = servo.build_command()
        #ser.write(bytes(cmd))
        print(f"[初始化] 舵机{servo.servo_id} → 初始角度 {servo.init_angle}°，命令: {[hex(b) for b in cmd]}")

        # 初始化水泵和照明灯（不包含DVL）
    for p in peripherals:
        if p['id'] != 0x07:  # 跳过DVL
            cmd = [0xAA, p['id'], p['speed'], 0x00, 0x00]
            crc = crc8_func(bytes(cmd))
            cmd.append(crc)
            #ser.write(bytes(cmd))
            print(f"[初始化] {p['name']} → 初始转速 {p['speed']}，命令: {[hex(b) for b in cmd]}")

# ================== 主窗口 ==================
class ControlWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("舵机 & 设备控制系统")
        self.setGeometry(200, 200, 600, 500)

        # 串口
        self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.5)

        # 4 个舵机
        self.servos = {
            1: Servo(1, init_angle=110, speed=20, step=5, angle_min=110, angle_max=260),
            2: Servo(2, init_angle=130, speed=20, step=5, angle_min=85, angle_max=260),
            3: Servo(3, init_angle=45, speed=35, step=5, angle_min=45, angle_max=260),
            4: Servo(4, init_angle=110, speed=20, step=5, angle_min=110, angle_max=180)
        }

        # 三个设备
        self.pump = Device(0x05)
        self.light = Device(0x06)
        self.dvl = Device(0x07)

        # GUI 元素
        self.layout = QVBoxLayout()

        # 舵机状态标签
        self.servo_labels = {}
        for i in range(1, 5):
            lbl = QLabel(f"舵机{i}: 角度 {self.servos[i].angle}°, 速度 {self.servos[i].speed}")
            self.servo_labels[i] = lbl
            self.layout.addWidget(lbl)

        # 动作组按钮
        self.layout.addWidget(QLabel("动作组:"))
        self.action_buttons = {}
        for i in range(1, 6):
            btn = QPushButton(f"{i} - 动作组 {i}")
            btn.clicked.connect(lambda _, x=i: self.run_action_group(x))
            self.action_buttons[i] = btn
            self.layout.addWidget(btn)

        # 水泵控制
        self.layout.addWidget(QLabel("水泵速度:"))
        pump_slider = QSlider(Qt.Horizontal)
        pump_slider.setRange(0, 99)
        pump_slider.valueChanged.connect(lambda val: self.update_device(self.pump, val))
        self.layout.addWidget(pump_slider)

        # 照明灯控制
        self.layout.addWidget(QLabel("照明灯亮度:"))
        light_slider = QSlider(Qt.Horizontal)
        light_slider.setRange(0, 99)
        light_slider.valueChanged.connect(lambda val: self.update_device(self.light, val))
        self.layout.addWidget(light_slider)

        # DVL 控制
        self.layout.addWidget(QLabel("DVL 开关:"))
        self.dvl_button = QPushButton("关闭")
        self.dvl_button.setCheckable(True)
        self.dvl_button.clicked.connect(self.toggle_dvl)
        self.layout.addWidget(self.dvl_button)

        # 命令显示
        self.cmd_label = QLabel("最近发送命令: ")
        self.layout.addWidget(self.cmd_label)

        # 设置中央窗口
        central_widget = QWidget()
        central_widget.setLayout(self.layout)
        self.setCentralWidget(central_widget)

        # 初始化舵机到初始角度
        self.init_servos()

    def init_servos(self):
        for i in self.servos:
            self.send_command(self.servos[i].build_command())
            self.update_servo_label(i)

    def send_command(self, cmd):
        self.ser.write(bytes(cmd))
        hex_cmd = " ".join(f"{b:02X}" for b in cmd)
        self.cmd_label.setText(f"最近发送命令: {hex_cmd}")

    def update_servo_label(self, servo_id):
        servo = self.servos[servo_id]
        self.servo_labels[servo_id].setText(f"舵机{servo_id}: 角度 {servo.angle}°, 速度 {servo.speed}")

    def update_device(self, device, val):
        device.set_value(val)
        self.send_command(device.build_command())

    def toggle_dvl(self):
        if self.dvl_button.isChecked():
            self.dvl.set_value(0x99)
            self.dvl_button.setText("开启")
        else:
            self.dvl.set_value(0x00)
            self.dvl_button.setText("关闭")
        self.send_command(self.dvl.build_command())

    def run_action_group(self, group_id):
        angles = {
            1: [100, 120, 90, 80],
            2: [120, 90, 100, 90],
            3: [150, 150, 150, 150],
            4: [90, 90, 90, 90],
            5: [180, 180, 180, 180]
        }
        for i, angle in enumerate(angles[group_id], start=1):
            self.servos[i].set_angle(angle)
            self.send_command(self.servos[i].build_command())
            self.update_servo_label(i)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_J:
            self.servos[1].increase_angle()
            self.send_command(self.servos[1].build_command())
            self.update_servo_label(1)
        elif event.key() == Qt.Key_K:
            self.servos[1].decrease_angle()
            self.send_command(self.servos[1].build_command())
            self.update_servo_label(1)

        elif event.key() == Qt.Key_A:
            self.servos[2].decrease_angle()
            self.send_command(self.servos[2].build_command())
            self.update_servo_label(2)
        elif event.key() == Qt.Key_D:
            self.servos[2].increase_angle()
            self.send_command(self.servos[2].build_command())
            self.update_servo_label(2)

        elif event.key() == Qt.Key_W:
            self.servos[3].increase_angle()
            self.send_command(self.servos[3].build_command())
            self.update_servo_label(3)
        elif event.key() == Qt.Key_S:
            self.servos[3].decrease_angle()
            self.send_command(self.servos[3].build_command())
            self.update_servo_label(3)

        elif event.key() == Qt.Key_Q:
            self.servos[4].decrease_angle()
            self.send_command(self.servos[4].build_command())
            self.update_servo_label(4)
        elif event.key() == Qt.Key_E:
            self.servos[4].increase_angle()
            self.send_command(self.servos[4].build_command())
            self.update_servo_label(4)

# ================== 主函数 ==================
def main():
    ser = serial.Serial("/dev/ttyUSB0", 115200)

    # 舵机初始化
    servos = {
        1: Servo(1, init_angle=110, speed=20, step=5, angle_min=110, angle_max=260),
        2: Servo(2, init_angle=130, speed=20, step=5, angle_min=85, angle_max=260),
        3: Servo(3, init_angle=45, speed=35, step=5, angle_min=45, angle_max=260),
        4: Servo(4, init_angle=110, speed=20, step=5, angle_min=110, angle_max=180)
    }

    # 外设初始化
    peripherals = [
        {'id': 0x05, 'name': '水泵', 'speed': 50},
        {'id': 0x06, 'name': '照明灯', 'speed': 0},
        {'id': 0x07, 'name': 'DVL', 'speed': 0}  # DVL不初始化
    ]

    # 初始化所有设备（除了DVL）
    initialize_devices(ser, servos, peripherals)

    app = QApplication(sys.argv)
    window = ControlWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
