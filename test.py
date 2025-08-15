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
INIT_GAP_MS = 120          # 初始化阶段每个数据包的间隔
PROCESS_STEP_GAP_MS = 80   # 动作组内（同一组里不同舵机）每个数据包的间隔

# 三个流程：列表里每一项是 (角度字典, 延时秒) —— 你可在此自由修改测试值
PROCESS_ACTIONS: Dict[str, List[Tuple[Dict[int, int], float]]] = {
    "准备抓取姿态0.5s": [
        ({2: 85, 3:  260, 4:  180}, 0.5),
    ],

    "开始抓取10s": [

        ({2: 85,  3:  45, 4: 110}, 3.0),#爪子闭合
        ({2: 260, 3:  45, 4: 110}, 2.0),#大臂回笼
        #甩干模式
        ({2: 200, 3:  45, 4:  110}, 0.3),
        ({2: 240, 3:  45, 4:  110}, 0.3),
        ({2: 200, 3:  45, 4:  110}, 0.3),
        ({2: 240, 3:  45, 4:  110}, 0.3),
        ({2: 200, 3:  45, 4:  110}, 0.3),
        #进入观察模式
        ({2: 175, 3:  45,  4:  110}, 2),
        ({2: 175, 3:  260, 4: 180}, 0.5),
    ],

    "观察者模式": [
        ({2: 175, 3:  260, 4: 180}, 0.5),
    ],

}

# 舵机初始参数（每个舵机都有独立角度限制）
SERVOS_CONFIG = {
    1: dict(name="相机云台舵机1", init_angle=110, speed=20, step=5, angle_min=110, angle_max=260),
    2: dict(name="机械臂大臂舵机2", init_angle=130, speed=30, step=5, angle_min=85, angle_max=260),
    3: dict(name="笼子开关舵机3", init_angle=45, speed=35, step=5, angle_min=45, angle_max=260),
    4: dict(name="爪子舵机4", init_angle=110, speed=20, step=5, angle_min=110, angle_max=180)
}

# 外设初始值（DVL 不初始化）
PUMP_INIT  = 50   # 水泵 ID 0x05
LIGHT_INIT = 0   # 照明 ID 0x06


# ========================= CRC8（使用 crcmod） =========================
# 预设 'crc-8'：poly=0x07, init=0x00, refin=False, refout=False, xorout=0x00
CRC8_FUNC = crcmod.predefined.mkPredefinedCrcFun('crc-8')

def crc8_full(frame_no_crc: List[int]) -> int:
    """对帧的所有字节（含0xAA帧头，不含CRC自身）做CRC8，返回0~255。"""
    return CRC8_FUNC(bytes(frame_no_crc))

def hex_str(bs: List[int]) -> str:
    return " ".join(f"{b:02X}" for b in bs)


# ========================= 串口工具（含虚拟串口） =========================
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
        if ("USB" in p.device.upper()) or ("ACM" in p.device.upper()) or ("COM" in p.device.upper()):
            chosen = p.device
            break
    if chosen is None and ports:
        chosen = ports[0].device  # 兜底

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
      - 角度低位在前 (low = angle >> 8)，高位在后
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

    def move_by_step(self, direction: int) -> int:
        """direction: +1/-1"""
        return self.set_angle(self.angle + direction * self.step)

    def build_frame(self, angle_val: int = None) -> List[int]:
        if angle_val is None:
            angle_val = self.angle
        angle_val = self.clamp(angle_val)
        # 低位在前、高位在后
        low  = (angle_val >> 8) & 0xFF
        high =  angle_val       & 0xFF
        frame = [0xAA, self.id, high, low, self.speed]
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

        # 键盘长按状态 —— 新增（J/K 控 1；W/S 控 2；A/D 控 3；Q/E 控 4）
        self.key_states: Dict[int, bool] = {
            Qt.Key_J: False, Qt.Key_K: False,  # servo1
            Qt.Key_W: False, Qt.Key_S: False,  # servo2
            Qt.Key_A: False, Qt.Key_D: False,  # servo3
            Qt.Key_Q: False, Qt.Key_E: False,  # servo4
        }
        # 键位到（舵机ID, 方向）映射：第一键为减(-1)，第二键为加(+1)
        self.key_map: Dict[int, Tuple[int,int]] = {
            Qt.Key_J: (1, -1), Qt.Key_K: (1, +1),
            Qt.Key_W: (2, +1), Qt.Key_S: (2, -1),
            Qt.Key_A: (3, -1), Qt.Key_D: (3, +1),
            Qt.Key_Q: (4, -1), Qt.Key_E: (4, +1),
        }

        self._build_ui()

        # 启动时发送初始值并同步 GUI（DVL 不初始化）
        QtCore.QTimer.singleShot(100, self._send_initial_values_and_sync)

        # 定时器：处理键盘长按（50ms）
        self.key_timer = QtCore.QTimer(self)
        self.key_timer.timeout.connect(self._on_key_timer)
        self.key_timer.start(50)

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

        # 初始化表
        for row, sid in enumerate(sorted(self.servos.keys())):
            s = self.servos[sid]
            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(s.name))
            self.table.setItem(row, 1, QtWidgets.QTableWidgetItem(f"{s.angle}°"))
            self.table.setItem(row, 2, QtWidgets.QTableWidgetItem(str(s.speed)))
            self.table.setItem(row, 3, QtWidgets.QTableWidgetItem(f"{s.angle_min}~{s.angle_max}"))
            self.table.setItem(row, 4, QtWidgets.QTableWidgetItem("--"))

        # ⭐ 新增：键位备注（放在舵机状态区域下方）
        self.key_hint = QtWidgets.QLabel("键位：J/K→舵机1（减/加），W/S→舵机2（加/减），A/D→舵机3（减/加），Q/E→舵机4（减/加）  （按住可连续控制）")
        self.key_hint.setStyleSheet("color:#666;")
        left.addWidget(self.key_hint)

        top.addLayout(left, 3)

        # 中：三个流程按钮（保持不变）
        mid = QtWidgets.QVBoxLayout()
        mid.addWidget(self._title("流程控制（可在代码顶部修改角度与间隔）"))
        self.proc_btns = {}
        for i, pname in enumerate(PROCESS_ACTIONS.keys(), start=1):
            btn = QtWidgets.QPushButton(f"{pname}")
            btn.setFixedHeight(40)
            btn.clicked.connect(lambda check=False, n=pname: self._run_process(n))
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

        # DVL 开关（明确状态，颜色指示）
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

    def _send_sequence_with_gap(self, items, gap_ms: int, done_cb=None):
        """
        顺序发送 (frame, ui_callback) 列表；每帧间隔 gap_ms 毫秒。
        仅 Linux 版：所有回调按“无参”调用。
        """
        if not items:
            if callable(done_cb):
                try:
                    done_cb()  # 无参调用
                except TypeError:
                    pass
            return

        frame, cb = items.pop(0)

        # 发送 + 日志
        self._send(frame)
        self._log(hex_str(frame))

        # UI 回调（无参）
        if callable(cb):
            try:
                cb()
            except TypeError:
                pass

        # 定时下一帧（无参 lambda）
        QtCore.QTimer.singleShot(int(gap_ms), lambda: self._send_sequence_with_gap(items, gap_ms, done_cb))

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
        """
        启动时的初始化：按顺序发送（每帧之间留 INIT_GAP_MS 间隔）
          1) 舵机：各自 init_angle
          2) 水泵：初始转速
          3) 照明：初始亮度
        DVL 不初始化
        """
        # 先把滑条值就位，但暂时不触发 valueChanged（避免重复发）
        self.pump_slider.blockSignals(True)
        self.light_slider.blockSignals(True)
        self.pump_slider.setValue(self.dev_pump.value)
        self.light_slider.setValue(self.dev_light.value)
        self.pump_slider.blockSignals(False)
        self.light_slider.blockSignals(False)

        # 构建要发送的初始化帧队列（每项是 (frame, callback)）
        sequence = []

        # 1) 舵机：按 ID 顺序
        for sid in sorted(self.servos.keys()):
            s = self.servos[sid]
            s.set_angle(s.init_angle)
            frame = s.build_frame()

            # 发送该帧后刷新表格一行（为避免频繁刷新，可只设置 last_hex 字段）
            def mk_servo_cb(_sid=sid):
                return lambda: self._refresh_servo_table()

            sequence.append((frame, mk_servo_cb()))

        # 2) 水泵初始帧
        f_pump = self.dev_pump.build_frame(self.dev_pump.value)

        def pump_cb():
            self.pump_cmd.setText("命令: " + hex_str(f_pump))

        sequence.append((f_pump, pump_cb))

        # 3) 照明初始帧
        f_light = self.dev_light.build_frame(self.dev_light.value)

        def light_cb():
            self.light_cmd.setText("命令: " + hex_str(f_light))

        sequence.append((f_light, light_cb))

        # 依次发送，帧间隔 INIT_GAP_MS
        self._send_sequence_with_gap(sequence, INIT_GAP_MS)

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

    # --------------------- 键盘处理（长按） ---------------------
    def keyPressEvent(self, event):
        key = event.key()
        if key in self.key_states:
            self.key_states[key] = True

    def keyReleaseEvent(self, event):
        key = event.key()
        if key in self.key_states:
            self.key_states[key] = False

    def _on_key_timer(self):
        moved = False
        for key, pressed in self.key_states.items():
            if not pressed:
                continue
            sid, direction = self.key_map[key]
            s = self.servos[sid]
            s.move_by_step(direction)
            f = s.build_frame()
            self._send(f); self._log(hex_str(f))
            moved = True
        if moved:
            self._refresh_servo_table()

    # --------------------- 三个流程（保持不变） ---------------------

    def _run_process(self, pname: str):
        steps = PROCESS_ACTIONS.get(pname, [])
        if not steps:
            return
        self._log(f"触发 {pname} 开始")
        # 禁用按钮避免并发执行
        for b in self.proc_btns.values():
            b.setEnabled(False)

        # 用 QTimer 顺序执行（避免阻塞 UI）
        self._process_queue = list(steps)
        self._run_next_step()

    def _run_next_step(self):
        # 流程结束：恢复按钮
        if not hasattr(self, "_process_queue") or not self._process_queue:
            self._log("流程完成")
            for b in self.proc_btns.values():
                b.setEnabled(True)
            return

        # 取出当前这一组：angles 是 {sid: angle}，delay_sec 是组与组之间的间隔
        angles, delay_sec = self._process_queue.pop(0)

        # 组内帧队列：每个舵机一帧 + 对应的 UI 刷新回调
        seq = []
        for sid, ang in angles.items():
            if sid not in self.servos:
                continue
            s = self.servos[sid]
            s.set_angle(ang)
            frame = s.build_frame()
            # 每发一帧后刷新表格，让“最后命令(HEX)”与角度立即更新
            seq.append((frame, self._refresh_servo_table))

        # 先按 PROCESS_STEP_GAP_MS 逐帧发送完“本组”的所有舵机命令，
        # 再等待 delay_sec，进入下一组
        def after_group_sent():
            QtCore.QTimer.singleShot(int(delay_sec * 1000), self._run_next_step)

        self._send_sequence_with_gap(seq, PROCESS_STEP_GAP_MS, done_cb=after_group_sent)


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
