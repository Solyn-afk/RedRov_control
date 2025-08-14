# -*- coding: utf-8 -*-
import sys
import time
from typing import Dict, List, Tuple

from PySide6 import QtWidgets, QtCore
from PySide6.QtCore import Qt

import serial
from serial.tools import list_ports
import crcmod.predefined


# ========================= 可配置区域 =========================
BAUDRATE = 115200

# 三个流程：列表里每一项是 (角度字典, 延时秒)
# 你可直接修改这些测试值
PROCESS_ACTIONS: Dict[str, List[Tuple[Dict[int, int], float]]] = {
    "流程1": [
        ({1: 100, 2: 120, 3:  80, 4:  90}, 0.8),
        ({1: 110, 2:  90, 3: 100, 4: 120}, 0.6),
        ({1:  90, 2: 100, 3:  90, 4:  90}, 0.5),
    ],
    "流程2": [
        ({1: 130, 2: 100, 3: 120, 4: 110}, 1.0),
        ({1: 120, 2: 110, 3:  90, 4:  80}, 0.7),
    ],
    "流程3": [
        ({1:  95, 2: 110, 3: 130, 4:  60}, 0.5),
        ({1: 140, 2:  80, 3: 110, 4: 100}, 0.9),
        ({1: 100, 2: 100, 3: 100, 4: 100}, 0.5),
    ],
}

# 舵机初始参数（你可修改）
SERVOS_CONFIG = {
    1: dict(name="舵机1", init_angle=90, speed=12, step=2, angle_min=0,  angle_max=180),
    2: dict(name="舵机2", init_angle=95, speed=15, step=3, angle_min=10, angle_max=160),
    3: dict(name="舵机3", init_angle=80, speed=10, step=1, angle_min=30, angle_max=180),
    4: dict(name="舵机4", init_angle=60, speed=18, step=2, angle_min=45, angle_max=135),
}

# 外设初始值（DVL 不初始化）
PUMP_INIT  = 30   # 水泵 ID 0x05
LIGHT_INIT = 50   # 照明 ID 0x06


# ========================= CRC8（使用 crcmod） =========================
# 采用预设 'crc-8'：poly=0x07, init=0x00, refin=False, refout=False, xorout=0x00
CRC8_FUNC = crcmod.predefined.mkPredefinedCrcFun('crc-8')

def crc8_full(frame_no_crc: List[int]) -> int:
    """对帧的所有字节（含0xAA帧头）做CRC8，返回0~255。"""
    return CRC8_FUNC(bytes(frame_no_crc))

def hex_str(bs: List[int]) -> str:
    return " ".join(f"{b:02X}" for b in bs)


# ========================= 串口工具 =========================
class VirtualSerial:
    """虚拟串口：write() 只打印，不实际发送"""
    def __init__(self, logger_cb=None):
        self.logger_cb = logger_cb

    def write(self, data: bytes):
        s = " ".join(f"{b:02X}" for b in data)
        print("[VIRTUAL SERIAL] ->", s)
        if self.logger_cb:
            self.logger_cb(s)

def open_serial_auto() -> tuple:
    """
    自动检测串口：
    - Windows: COM*
    - Linux: /dev/ttyUSB* / /dev/ttyACM*
    找到首个可用端口并打开；找不到则返回 VirtualSerial。
    """
    ports = list(list_ports.comports())
    chosen = None
    for p in ports:
        # 任何串口都可以，优先包含 USB/ACM 的
        if ("USB" in p.device.upper()) or ("ACM" in p.device.upper()) or ("COM" in p.device.upper()):
            chosen = p.device
            break
    if chosen is None and ports:
        chosen = ports[0].device  # 兜底使用第一个

    if chosen:
        try:
            ser = serial.Serial(chosen, BAUDRATE, timeout=0.1)
            print(f"[SERIAL] 打开串口：{chosen} @ {BAUDRATE}")
            return ser, chosen, False
        except Exception as e:
            print(f"[WARN] 串口打开失败：{chosen} -> {e}")

    print("[SERIAL] 未检测到可用串口，进入虚拟串口模式。")
    return VirtualSerial(), "VIRTUAL", True


# ========================= 舵机类 =========================
class Servo:
    """
    舵机指令：0xAA, id, angle_low, angle_high, speed, CRC8
      - 角度低位在前 (low = angle >> 8)
      - CRC8 对前 5 个字节（含 0xAA）计算
    """
    def __init__(self, servo_id: int, name: str,
                 init_angle: int, speed: int, step: int,
                 angle_min: int, angle_max: int):
        self.id = servo_id
        self.name = name
        self.init_angle = int(init_angle)
        self.speed = int(speed)
        self.step = int(step)
        self.angle_min = int(angle_min)
        self.angle_max = int(angle_max)
        self.angle = int(init_angle)
        self.last_hex = "--"

    def clamp(self, a: int) -> int:
        return max(self.angle_min, min(self.angle_max, int(a)))

    def set_angle(self, a: int) -> int:
        self.angle = self.clamp(a)
        return self.angle

    def build_frame(self, angle_val: int = None) -> List[int]:
        if angle_val is None:
            angle_val = self.angle
        angle_val = self.clamp(angle_val)
        # 低位在前、高位在后
        low  = (angle_val >> 8) & 0xFF
        high =  angle_val       & 0xFF
        frame = [0xAA, self.id, low, high, self.speed]
        c = crc8_full(frame)
        frame.append(c)
        self.last_hex = hex_str(frame)
        return frame


# ========================= 外设类 =========================
class Device:
    """
    外设（水泵/照明/DVL）
    帧：0xAA, id, value, 0x00, 0x00, CRC8
      - 水泵/照明 value: 0-99
      - DVL value: 0x00(关) / 0x99(开)
      - CRC8 对前 5 个字节（含 0xAA）计算
    """
    def __init__(self, dev_id: int, name: str, init_value: int = 0, vmin: int = 0, vmax: int = 99):
        self.id = dev_id
        self.name = name
        self.vmin = vmin
        self.vmax = vmax
        self.value = self._clip(init_value)
        self.last_hex = "--"

    def _clip(self, v: int) -> int:
        return max(self.vmin, min(self.vmax, int(v)))

    def set_value(self, v: int) -> int:
        self.value = self._clip(v)
        return self.value

    def build_frame(self, value: int = None) -> List[int]:
        if value is None:
            value = self.value
        v = self._clip(value)
        frame = [0xAA, self.id, v & 0xFF, 0x00, 0x00]
        c = crc8_full(frame)
        frame.append(c)
        self.last_hex = hex_str(frame)
        return frame


# ========================= 主窗口 =========================
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("舵机 & 外设控制（PySide6 + 虚拟串口）")
        self.resize(1000, 640)

        # 串口自动检测；若失败，使用虚拟串口
        self.ser, self.port_name, self.virtual_mode = open_serial_auto()

        # 舵机
        self.servos: Dict[int, Servo] = {
            sid: Servo(sid, cfg["name"], cfg["init_angle"], cfg["speed"], cfg["step"],
                       cfg["angle_min"], cfg["angle_max"])
            for sid, cfg in SERVOS_CONFIG.items()
        }

        # 外设
        self.dev_pump  = Device(0x05, "水泵",  init_value=PUMP_INIT)
        self.dev_light = Device(0x06, "照明灯", init_value=LIGHT_INIT)
        # DVL 不初始化；0x00=关，0x99=开
        self.dev_dvl   = Device(0x07, "DVL",   init_value=0, vmin=0, vmax=0x99)

        self._build_ui()

        # 启动时发送初始值并同步 GUI（DVL 不初始化）
        QtCore.QTimer.singleShot(100, self._send_initial_values_and_sync)

    # --------------------- UI ---------------------
    def _build_ui(self):
        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        self.setCentralWidget(central)

        # 顶部串口状态
        self.port_label = QtWidgets.QLabel(
            f"串口：{self.port_name} ({'虚拟' if self.virtual_mode else '真实'})"
        )
        root.addWidget(self.port_label)

        top = QtWidgets.QHBoxLayout()
        root.addLayout(top, 5)

        # 左：舵机表
        left = QtWidgets.QVBoxLayout()
        left.addWidget(self._title("舵机状态"))
        self.table = QtWidgets.QTableWidget(4, 5)
        self.table.setHorizontalHeaderLabels(["舵机", "当前角度", "速度", "角度限制", "最后命令(HEX)"])
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        left.addWidget(self.table)
        top.addLayout(left, 3)

        # 初始化表
        for row, sid in enumerate(sorted(self.servos.keys())):
            s = self.servos[sid]
            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(s.name))
            self.table.setItem(row, 1, QtWidgets.QTableWidgetItem(f"{s.angle}°"))
            self.table.setItem(row, 2, QtWidgets.QTableWidgetItem(str(s.speed)))
            self.table.setItem(row, 3, QtWidgets.QTableWidgetItem(f"{s.angle_min}~{s.angle_max}"))
            self.table.setItem(row, 4, QtWidgets.QTableWidgetItem("--"))

        # 中：三个流程按钮
        mid = QtWidgets.QVBoxLayout()
        mid.addWidget(self._title("流程控制（可在代码顶部修改角度与间隔）"))
        self.proc_btns = {}
        for i, pname in enumerate(PROCESS_ACTIONS.keys(), start=1):
            btn = QtWidgets.QPushButton(f"{pname}")
            btn.setFixedHeight(40)
            btn.clicked.connect(lambda _, n=pname: self._run_process(n))
            self.proc_btns[pname] = btn
            mid.addWidget(btn)
        mid.addStretch()
        top.addLayout(mid, 2)

        # 右：外设
        right = QtWidgets.QVBoxLayout()
        right.addWidget(self._title("外设控制"))

        # 水泵
        right.addWidget(self._subtitle("水泵 (ID 0x05)"))
        self.pump_slider = QtWidgets.QSlider(Qt.Horizontal)
        self.pump_slider.setRange(0, 99)
        self.pump_slider.valueChanged.connect(self._on_pump_changed)
        right.addWidget(self.pump_slider)
        self.pump_cmd = QtWidgets.QLabel("命令: --")
        right.addWidget(self.pump_cmd)

        # 照明灯
        right.addWidget(self._subtitle("照明灯 (ID 0x06)"))
        self.light_slider = QtWidgets.QSlider(Qt.Horizontal)
        self.light_slider.setRange(0, 99)
        self.light_slider.valueChanged.connect(self._on_light_changed)
        right.addWidget(self.light_slider)
        self.light_cmd = QtWidgets.QLabel("命令: --")
        right.addWidget(self.light_cmd)

        # DVL 开关（明确状态）
        right.addWidget(self._subtitle("DVL (ID 0x07) 开/关"))
        self.dvl_btn = QtWidgets.QPushButton("关闭")
        self.dvl_btn.setCheckable(True)
        self.dvl_btn.setChecked(False)
        self._refresh_dvl_visual(False)
        self.dvl_btn.clicked.connect(self._on_dvl_toggled)
        right.addWidget(self.dvl_btn)
        self.dvl_cmd = QtWidgets.QLabel("命令: --")
        right.addWidget(self.dvl_cmd)

        right.addStretch()
        top.addLayout(right, 2)

        # 底部日志
        root.addWidget(self._title("发送命令日志（最近 12 条）"))
        self.log = QtWidgets.QTextEdit()
        self.log.setReadOnly(True)
        self.log.setFixedHeight(150)
        root.addWidget(self.log, 2)

    def _title(self, txt):
        lbl = QtWidgets.QLabel(txt)
        f = lbl.font(); f.setPointSize(13); f.setBold(True)
        lbl.setFont(f)
        return lbl

    def _subtitle(self, txt):
        lbl = QtWidgets.QLabel(txt)
        f = lbl.font(); f.setPointSize(11); f.setBold(True)
        lbl.setFont(f)
        return lbl

    # --------------------- 初始化发送并同步 GUI ---------------------
    def _send_initial_values_and_sync(self):
        # 同步滑条，避免初始化时 valueChanged 触发重复发送
        self.pump_slider.blockSignals(True)
        self.light_slider.blockSignals(True)

        # 舵机初始角度
        for sid, s in self.servos.items():
            s.set_angle(s.init_angle)
            frame = s.build_frame()
            self._send(frame)
            self._log(hex_str(frame))
        self._refresh_servo_table()

        # 水泵/照明初始化 + 同步滑条
        self.pump_slider.setValue(self.dev_pump.value)
        f1 = self.dev_pump.build_frame(self.dev_pump.value)
        self._send(f1); self._log(hex_str(f1)); self.pump_cmd.setText("命令: " + hex_str(f1))

        self.light_slider.setValue(self.dev_light.value)
        f2 = self.dev_light.build_frame(self.dev_light.value)
        self._send(f2); self._log(hex_str(f2)); self.light_cmd.setText("命令: " + hex_str(f2))

        self.pump_slider.blockSignals(False)
        self.light_slider.blockSignals(False)
        # DVL 不初始化

    # --------------------- 表格刷新 & 日志 & 发送 ---------------------
    def _refresh_servo_table(self):
        for row, sid in enumerate(sorted(self.servos.keys())):
            s = self.servos[sid]
            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(s.name))
            self.table.setItem(row, 1, QtWidgets.QTableWidgetItem(f"{s.angle}°"))
            self.table.setItem(row, 2, QtWidgets.QTableWidgetItem(str(s.speed)))
            self.table.setItem(row, 3, QtWidgets.QTableWidgetItem(f"{s.angle_min}~{s.angle_max}"))
            self.table.setItem(row, 4, QtWidgets.QTableWidgetItem(s.last_hex))

    def _send(self, frame: List[int]):
        try:
            self.ser.write(bytes(frame))
        except Exception as e:
            print(f"[WARN] 发送失败：{e}")

    def _log(self, text: str):
        ts = time.strftime("%H:%M:%S")
        lines = self.log.toPlainText().splitlines()
        lines.append(f"[{ts}] {text}")
        if len(lines) > 12:
            lines = lines[-12:]
        self.log.setPlainText("\n".join(lines))
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    # --------------------- 外设回调 ---------------------
    def _on_pump_changed(self, v: int):
        self.dev_pump.set_value(v)
        f = self.dev_pump.build_frame()
        self._send(f); self._log(hex_str(f))
        self.pump_cmd.setText("命令: " + hex_str(f))

    def _on_light_changed(self, v: int):
        self.dev_light.set_value(v)
        f = self.dev_light.build_frame()
        self._send(f); self._log(hex_str(f))
        self.light_cmd.setText("命令: " + hex_str(f))

    def _on_dvl_toggled(self, checked: bool):
        # 开=0x99，关=0x00
        self.dev_dvl.set_value(0x99 if checked else 0x00)
        f = self.dev_dvl.build_frame()
        self._send(f); self._log(hex_str(f))
        self.dvl_cmd.setText("命令: " + hex_str(f))
        self._refresh_dvl_visual(checked)

    def _refresh_dvl_visual(self, on: bool):
        self.dvl_btn.setText("开启" if on else "关闭")
        self.dvl_btn.setStyleSheet(
            "QPushButton { font-weight: bold; color: white; background-color: %s; padding: 6px; }"
            % ("#2e7d32" if on else "#c62828")
        )

    # --------------------- 三个流程 ---------------------
    def _run_process(self, pname: str):
        steps = PROCESS_ACTIONS.get(pname, [])
        if not steps:
            return
        self._log(f"触发 {pname} 开始")
        # 禁用按钮避免并发执行
        for b in self.proc_btns.values():
            b.setEnabled(False)

        # 用 QTimer 顺序执行（避免阻塞 UI）
        self._process_queue = list(steps)  # 拷贝
        self._run_next_step()

    def _run_next_step(self):
        if not self._process_queue:
            self._log("流程完成")
            for b in self.proc_btns.values():
                b.setEnabled(True)
            return

        angles, delay_sec = self._process_queue.pop(0)
        # 发送本组角度
        for sid, ang in angles.items():
            if sid in self.servos:
                s = self.servos[sid]
                s.set_angle(ang)
                f = s.build_frame()
                self._send(f); self._log(hex_str(f))
        self._refresh_servo_table()

        # 等待 delay_sec 后继续下一组
        QtCore.QTimer.singleShot(int(delay_sec * 1000), self._run_next_step)


# ========================= 入口 =========================
def main():
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()

    # 如果是虚拟串口，把日志回调绑定上（可选）
    if isinstance(win.ser, VirtualSerial):
        win.ser.logger_cb = win._log

    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
