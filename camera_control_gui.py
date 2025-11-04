#!/usr/bin/env python3
"""
Camera Control GUI (ROS 2 + PySide6)

Improvements:
- "Setup prefix" field to source ROS/WS before commands (persisted to JSON)
- Per-row live log console (stdout/stderr)
- Status & exit-code reporting
- Fixed default ros_gz_bridge example (closing bracket)
- "Check ROS" button to verify `ros2` availability

Usage:
  1) Run from a terminal, or fill the "Setup prefix" so each command sources your env.
  2) Edit topics/commands as needed, click "Apply Topics".
  3) Press Start and watch the per-row log.

Author: ChatGPT (2025-11-04)
"""
import os, sys, json, signal, subprocess
from dataclasses import dataclass
from typing import Optional, Dict

import numpy as np
import cv2
from PySide6 import QtCore, QtGui, QtWidgets

# ROS 2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

APP_NAME = "Camera Control GUI"
CONFIG_FILE = os.path.join(os.path.dirname(__file__), "camera_control_gui.config.json")

DEFAULT_CONFIG = {
    "topics": {
        "rgb": "/camera/rgb/image_raw",
        "depth": "/camera/depth/image_raw",
        "thermal": "/camera/thermal/image_raw"
    },
    "commands": {
        "Start Gazebo world": "ros2 launch navigation scout.launch.py",
        "Start ROS�GZ bridge (demo clock)": "ros2 run ros_gz_bridge parameter_bridge /world/demo/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]",
        "Start cameras launch": "ros2 launch perception cameras.launch.py",
        "Spawn animals": "ros2 run perception random_animal_spawner"
    },
    "setup_prefix": "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && "
}

# ---------- Utilities ----------
def load_config() -> Dict:
    try:
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                cfg = json.load(f)
                for k, v in DEFAULT_CONFIG.items():
                    if k not in cfg:
                        cfg[k] = v
                if "setup_prefix" not in cfg:
                    cfg["setup_prefix"] = DEFAULT_CONFIG["setup_prefix"]
                return cfg
    except Exception as e:
        print(f"[WARN] Failed to load config: {e}", file=sys.stderr)
    return DEFAULT_CONFIG.copy()

def save_config(cfg: Dict) -> None:
    try:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
    except Exception as e:
        print(f"[WARN] Failed to save config: {e}", file=sys.stderr)

def cv_to_qimage(img_bgr: np.ndarray) -> QtGui.QImage:
    if img_bgr is None:
        return QtGui.QImage()
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    h, w, ch = img_rgb.shape
    return QtGui.QImage(img_rgb.data, w, h, ch * w, QtGui.QImage.Format_RGB888)

def depth_to_bgr(depth: np.ndarray) -> np.ndarray:
    if depth is None:
        return None
    depth_float = depth.astype(np.float32)
    valid = np.isfinite(depth_float) & (depth_float > 0)
    if np.any(valid):
        dmin = float(np.nanmin(depth_float[valid]))
        dmax = float(np.nanmax(depth_float[valid]))
        dmax = dmin + 1.0 if (dmax - dmin) < 1e-6 else dmax
        norm = np.clip((depth_float - dmin) / (dmax - dmin), 0.0, 1.0)
    else:
        norm = np.zeros_like(depth_float, dtype=np.float32)
    gray = (norm * 255.0).astype(np.uint8)
    return cv2.applyColorMap(gray, cv2.COLORMAP_JET)

def thermal_to_bgr(thermal: np.ndarray) -> np.ndarray:
    if thermal is None:
        return None
    if thermal.dtype != np.uint8:
        th = thermal.astype(np.float32)
        valid = np.isfinite(th)
        if np.any(valid):
            tmin = float(np.nanmin(th[valid]))
            tmax = float(np.nanmax(th[valid]))
            tmax = tmin + 1.0 if (tmax - tmin) < 1e-6 else tmax
            norm = np.clip((th - tmin) / (tmax - tmin), 0.0, 1.0)
        else:
            norm = np.zeros_like(th, dtype=np.float32)
        th8 = (norm * 255.0).astype(np.uint8)
    else:
        th8 = thermal
    return cv2.applyColorMap(th8, cv2.COLORMAP_INFERNO)

# ---------- ROS Node ----------
class CameraNode(Node):
    def __init__(self, topic_rgb: str, topic_depth: str, topic_thermal: str):
        super().__init__("camera_control_gui_node")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5,
                         durability=DurabilityPolicy.VOLATILE)
        self.bridge = CvBridge()
        self.rgb_image_bgr = None
        self.depth_image_bgr = None
        self.thermal_image_bgr = None
        self.sub_rgb = self.create_subscription(Image, topic_rgb, self.cb_rgb, qos)
        self.sub_depth = self.create_subscription(Image, topic_depth, self.cb_depth, qos)
        self.sub_thermal = self.create_subscription(Image, topic_thermal, self.cb_thermal, qos)
        self.get_logger().info(f"RGB: {topic_rgb} | Depth: {topic_depth} | Thermal: {topic_thermal}")

    def update_topics(self, topic_rgb: str, topic_depth: str, topic_thermal: str):
        for sub in [self.sub_rgb, self.sub_depth, self.sub_thermal]:
            if sub: self.destroy_subscription(sub)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5,
                         durability=DurabilityPolicy.VOLATILE)
        self.sub_rgb = self.create_subscription(Image, topic_rgb, self.cb_rgb, qos)
        self.sub_depth = self.create_subscription(Image, topic_depth, self.cb_depth, qos)
        self.sub_thermal = self.create_subscription(Image, topic_thermal, self.cb_thermal, qos)
        self.get_logger().info(f"[Topics updated] RGB={topic_rgb}, Depth={topic_depth}, Thermal={topic_thermal}")

    def cb_rgb(self, msg: Image):
        try:
            self.rgb_image_bgr = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"RGB conversion failed: {e}")

    def cb_depth(self, msg: Image):
        try:
            cv_img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_image_bgr = depth_to_bgr(cv_img) if cv_img.ndim == 2 else cv_img
        except Exception as e:
            self.get_logger().warn(f"Depth conversion failed: {e}")

    def cb_thermal(self, msg: Image):
        try:
            cv_img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if cv_img.ndim == 2:
                self.thermal_image_bgr = thermal_to_bgr(cv_img)
            else:
                self.thermal_image_bgr = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR) if cv_img.shape[2] == 3 else cv_img
        except Exception as e:
            self.get_logger().warn(f"Thermal conversion failed: {e}")

# ---------- GUI Widgets ----------
class ImagePanel(QtWidgets.QGroupBox):
    def __init__(self, title: str):
        super().__init__(title)
        self.label = QtWidgets.QLabel("No Image")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setMinimumSize(320, 240)
        self.label.setStyleSheet("QLabel { background: #111; color: #aaa; border: 1px solid #444; }")
        lay = QtWidgets.QVBoxLayout(self); lay.addWidget(self.label)
    def update_image(self, img_bgr: Optional[np.ndarray]):
        if img_bgr is None: return
        qimg = cv_to_qimage(img_bgr)
        self.label.setPixmap(QtGui.QPixmap.fromImage(qimg).scaled(self.label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

class LogReader(QtCore.QThread):
    line_out = QtCore.Signal(str)
    def __init__(self, proc: subprocess.Popen):
        super().__init__()
        self.proc = proc
        self._running = True
    def run(self):
        if self.proc.stdout is None: return
        for line in iter(self.proc.stdout.readline, b''):
            if not self._running: break
            try:
                self.line_out.emit(line.decode('utf-8', errors='replace').rstrip('\n'))
            except Exception: pass
        try:
            self.proc.stdout.close()
        except Exception: pass
    def stop(self): self._running = False

class ProcessRow(QtWidgets.QWidget):
    def __init__(self, commands: Dict[str, str], get_setup_prefix_callable):
        super().__init__()
        self.proc: Optional[subprocess.Popen] = None
        self.reader: Optional[LogReader] = None
        self.get_setup_prefix = get_setup_prefix_callable

        self.combo = QtWidgets.QComboBox()
        for label, cmd in commands.items():
            self.combo.addItem(label, cmd)

        self.cmd_edit = QtWidgets.QLineEdit(self.combo.currentData())
        self.btn_start = QtWidgets.QPushButton("Start")
        self.btn_stop  = QtWidgets.QPushButton("Stop"); self.btn_stop.setEnabled(False)
        self.status = QtWidgets.QLabel("idle"); self.status.setMinimumWidth(140)
        self.log = QtWidgets.QTextEdit(); self.log.setReadOnly(True); self.log.setFixedHeight(120)

        hl = QtWidgets.QHBoxLayout()
        hl.addWidget(self.combo, 1); hl.addWidget(self.cmd_edit, 3)
        hl.addWidget(self.btn_start); hl.addWidget(self.btn_stop); hl.addWidget(self.status)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(hl); layout.addWidget(self.log)

        self.combo.currentIndexChanged.connect(lambda i: self.cmd_edit.setText(self.combo.itemData(i)))
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop.clicked.connect(self.on_stop)

    def append_log(self, text: str):
        self.log.append(text); self.log.moveCursor(QtGui.QTextCursor.End)

    def on_start(self):
        if self.proc and self.proc.poll() is None:
            self.status.setText("already running"); return
        cmd = self.cmd_edit.text().strip()
        if not cmd:
            self.status.setText("empty command"); return

        prefix = self.get_setup_prefix().strip()
        if prefix and not prefix.endswith('&&'):
            if not prefix.endswith('&& '):
                prefix += ' && '
        full_cmd = prefix + cmd if prefix else cmd

        try:
            self.proc = subprocess.Popen(
                ["bash", "-lc", full_cmd],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid, text=False, bufsize=1
            )
            self.status.setText("running"); self.btn_stop.setEnabled(True)
            self.append_log(f"$ {full_cmd}")
            self.reader = LogReader(self.proc); self.reader.line_out.connect(self.append_log); self.reader.start()

            self.watcher = QtCore.QTimer(self); self.watcher.timeout.connect(self._poll_status); self.watcher.start(500)
        except Exception as e:
            self.status.setText("error"); self.append_log(f"[error] {e}")

    def _poll_status(self):
        if self.proc and self.proc.poll() is not None:
            code = self.proc.returncode
            self.status.setText(f"exited {code}")
            self.btn_stop.setEnabled(False)
            if self.reader:
                self.reader.stop(); self.reader.wait(500); self.reader = None
            self.watcher.stop()

    def on_stop(self):
        if not self.proc:
            self.status.setText("no process"); return
        try:
            if self.proc.poll() is None:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                self.proc.wait(timeout=3)
            self.status.setText("stopped")
        except Exception as e:
            self.status.setText("stop error"); self.append_log(f"[stop error] {e}")
        finally:
            self.btn_stop.setEnabled(False)
            if self.reader:
                self.reader.stop(); self.reader.wait(500); self.reader = None
            self.proc = None

class MainWindow(QtWidgets.QWidget):
    def __init__(self, node: CameraNode, cfg: Dict):
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.node = node
        self.cfg = cfg

        # Panels
        self.rgb_panel = ImagePanel("RGB")
        self.depth_panel = ImagePanel("Depth")
        self.thermal_panel = ImagePanel("Thermal")

        # Topics
        topics = cfg.get("topics", {})
        self.edit_rgb = QtWidgets.QLineEdit(topics.get("rgb", DEFAULT_CONFIG["topics"]["rgb"]))
        self.edit_depth = QtWidgets.QLineEdit(topics.get("depth", DEFAULT_CONFIG["topics"]["depth"]))
        self.edit_thermal = QtWidgets.QLineEdit(topics.get("thermal", DEFAULT_CONFIG["topics"]["thermal"]))
        btn_apply_topics = QtWidgets.QPushButton("Apply Topics"); btn_apply_topics.clicked.connect(self.apply_topics)

        grid_topics = QtWidgets.QGridLayout()
        grid_topics.addWidget(QtWidgets.QLabel("RGB Topic"), 0, 0); grid_topics.addWidget(self.edit_rgb, 0, 1)
        grid_topics.addWidget(QtWidgets.QLabel("Depth Topic"), 1, 0); grid_topics.addWidget(self.edit_depth, 1, 1)
        grid_topics.addWidget(QtWidgets.QLabel("Thermal Topic"), 2, 0); grid_topics.addWidget(self.edit_thermal, 2, 1)
        grid_topics.addWidget(btn_apply_topics, 0, 2, 3, 1)
        topics_box = QtWidgets.QGroupBox("Topics"); topics_box.setLayout(grid_topics)

        # Setup prefix
        self.edit_setup = QtWidgets.QLineEdit(cfg.get("setup_prefix", DEFAULT_CONFIG["setup_prefix"]))
        btn_save_setup = QtWidgets.QPushButton("Save Prefix"); btn_save_setup.clicked.connect(self.save_setup)
        btn_check_ros = QtWidgets.QPushButton("Check ROS"); btn_check_ros.clicked.connect(self.check_ros)
        self.lbl_ros = QtWidgets.QLabel("")
        grid_setup = QtWidgets.QGridLayout()
        grid_setup.addWidget(QtWidgets.QLabel("Setup prefix (sourced before each command):"), 0, 0, 1, 3)
        grid_setup.addWidget(self.edit_setup, 1, 0, 1, 3)
        grid_setup.addWidget(btn_save_setup, 2, 0); grid_setup.addWidget(btn_check_ros, 2, 1); grid_setup.addWidget(self.lbl_ros, 2, 2)
        setup_box = QtWidgets.QGroupBox("Environment"); setup_box.setLayout(grid_setup)

        # Launch / commands
        self.rows_container = QtWidgets.QVBoxLayout(); self.rows_container.setSpacing(6)
        self.add_process_row()
        btn_add_row = QtWidgets.QPushButton("+ Add Row"); btn_add_row.clicked.connect(self.add_process_row)
        launch_box = QtWidgets.QGroupBox("Launch / Commands")
        vb_launch = QtWidgets.QVBoxLayout(launch_box)
        vb_launch.addLayout(self.rows_container); vb_launch.addWidget(btn_add_row, alignment=QtCore.Qt.AlignLeft)

        # Layout main
        img_layout = QtWidgets.QHBoxLayout()
        img_layout.addWidget(self.rgb_panel, 1); img_layout.addWidget(self.depth_panel, 1); img_layout.addWidget(self.thermal_panel, 1)

        root = QtWidgets.QVBoxLayout(self)
        root.addLayout(img_layout, 3)
        root.addWidget(topics_box, 0)
        root.addWidget(setup_box, 0)
        root.addWidget(launch_box, 0)

        # Timers
        self.timer = QtCore.QTimer(self); self.timer.setInterval(50); self.timer.timeout.connect(self.refresh_images); self.timer.start()
        self.spin_timer = QtCore.QTimer(self); self.spin_timer.setInterval(10); self.spin_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0.0)); self.spin_timer.start()

        self.setStyleSheet("""
        QWidget { background: #0B0F14; color: #D6DEE7; }
        QGroupBox { border: 1px solid #2E3B4E; border-radius: 8px; margin-top: 10px; padding: 8px; }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0px 3px; }
        QLineEdit { background: #0F151C; border: 1px solid #334155; border-radius: 6px; padding: 6px; }
        QPushButton { background: #14202B; border: 1px solid #334155; border-radius: 6px; padding: 6px 10px; }
        QPushButton:hover { background: #1B2A36; }
        QLabel { font-size: 13px; }
        QTextEdit { background: #0F151C; border: 1px solid #334155; border-radius: 6px; }
        """)

    def add_process_row(self):
        row = ProcessRow(self.cfg.get("commands", DEFAULT_CONFIG["commands"]), self.get_setup_prefix)
        self.rows_container.addWidget(row)

    def get_setup_prefix(self) -> str:
        return self.edit_setup.text()

    def save_setup(self):
        self.cfg["setup_prefix"] = self.edit_setup.text()
        save_config(self.cfg)

    def check_ros(self):
        cmd = self.edit_setup.text() + " ros2 --version"
        try:
            out = subprocess.check_output(["bash", "-lc", cmd], stderr=subprocess.STDOUT, timeout=5)
            self.lbl_ros.setText(out.decode("utf-8", errors="ignore").strip())
        except subprocess.CalledProcessError as e:
            self.lbl_ros.setText(f"error: {e.returncode}")
        except Exception as e:
            self.lbl_ros.setText(f"{e}")

    def apply_topics(self):
        rgb = self.edit_rgb.text().strip()
        depth = self.edit_depth.text().strip()
        thermal = self.edit_thermal.text().strip()
        self.node.update_topics(rgb, depth, thermal)
        self.cfg.setdefault("topics", {})
        self.cfg["topics"]["rgb"] = rgb; self.cfg["topics"]["depth"] = depth; self.cfg["topics"]["thermal"] = thermal
        save_config(self.cfg)

    def refresh_images(self):
        self.rgb_panel.update_image(self.node.rgb_image_bgr)
        self.depth_panel.update_image(self.node.depth_image_bgr)
        self.thermal_panel.update_image(self.node.thermal_image_bgr)

def main():
    cfg = load_config()
    rclpy.init(args=None)
    topics = cfg.get("topics", DEFAULT_CONFIG["topics"])
    node = CameraNode(topics.get("rgb", DEFAULT_CONFIG["topics"]["rgb"]),
                      topics.get("depth", DEFAULT_CONFIG["topics"]["depth"]),
                      topics.get("thermal", DEFAULT_CONFIG["topics"]["thermal"]))
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(node, cfg); win.resize(1200, 820); win.show()
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    ret = app.exec()
    node.destroy_node(); rclpy.shutdown(); sys.exit(ret)

if __name__ == "__main__":
    main()
