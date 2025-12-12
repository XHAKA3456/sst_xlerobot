#!/usr/bin/env python3
"""
Quest VR teleoperation for XLeRobot hardware - REFACTORED VERSION (No wheels)
하드웨어 직접 제어에 최적화 (MuJoCo 의존성 제거) + 베이스 모터 제거 버전
"""

import os
import sys
import time
import math
import socket
import struct
import threading
import queue
import logging
from pathlib import Path
from typing import Optional, Any

import cv2
import numpy as np

from sst_xlerbot.quest.quest_socket_monitor import QuestSocketMonitor
from sst_xlerbot.model.SO101Robot import SO101Kinematics

try:
    from lerobot.robots.xlerobot import XLerobotConfig, XLerobot
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.utils.errors import DeviceNotConnectedError
except ModuleNotFoundError:
    PROJECT_ROOT = Path(__file__).resolve().parents[4]
    LEROBOT_SRC = PROJECT_ROOT / "lerobot" / "src"
    if LEROBOT_SRC.exists():
        sys.path.insert(0, str(LEROBOT_SRC))
    from lerobot.robots.xlerobot import XLerobotConfig, XLerobot
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.utils.errors import DeviceNotConnectedError


VIDEO_SERVER_IP = "0.0.0.0"
VIDEO_PORT = 5656
CAMERA_INDEX = 2
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
JPEG_QUALITY = 85

logger = logging.getLogger(__name__)


class XLerobotArmOnly(XLerobot):
    """XLerobot wrapper that disconnects the omnidirectional base and optionally head motors."""

    def __init__(self, config: XLerobotConfig, use_head: bool = True):
        self.use_head = use_head
        super().__init__(config)

        # Re-create bus2 with only right-arm motors so missing wheel IDs are never queried.
        right_arm_motors = {
            name: motor for name, motor in self.bus2.motors.items() if name.startswith("right_arm")
        }
        calibration2 = None
        if self.bus2.calibration is not None:
            calibration2 = {k: v for k, v in self.bus2.calibration.items() if k in right_arm_motors}

        old_bus2 = self.bus2
        self.bus2 = FeetechMotorsBus(
            port=self.config.port2,
            motors=right_arm_motors,
            calibration=calibration2,
        )
        self.right_arm_motors = list(right_arm_motors.keys())
        self.base_motors = []
        del old_bus2

        if not self.use_head:
            left_arm_only = {
                name: motor for name, motor in self.bus1.motors.items() if name.startswith("left_arm")
            }
            calibration1 = None
            if self.bus1.calibration is not None:
                calibration1 = {k: v for k, v in self.bus1.calibration.items() if k in left_arm_only}

            old_bus1 = self.bus1
            self.bus1 = FeetechMotorsBus(
                port=self.config.port1,
                motors=left_arm_only,
                calibration=calibration1,
            )
            self.left_arm_motors = list(left_arm_only.keys())
            self.head_motors = []
            del old_bus1

    def _body_to_wheel_raw(self, *args, **kwargs):
        # Disable wheel commands entirely
        return {}

    def stop_base(self):
        # Base motors are not connected; nothing to stop.
        pass

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        left_arm_pos = self.bus1.sync_read("Present_Position", self.left_arm_motors)
        right_arm_pos = self.bus2.sync_read("Present_Position", self.right_arm_motors)
        head_state = {}
        if self.use_head:
            head_pos = self.bus1.sync_read("Present_Position", self.head_motors)
            head_state = {f"{k}.pos": v for k, v in head_pos.items()}

        left_arm_state = {f"{k}.pos": v for k, v in left_arm_pos.items()}
        right_arm_state = {f"{k}.pos": v for k, v in right_arm_pos.items()}
        base_vel = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}

        obs_dict = {**left_arm_state, **right_arm_state, **head_state, **base_vel}

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read arm/head state: {dt_ms:.1f}ms")

        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict


class QuestVideoStreamer:
    """Simple USB camera → Quest streamer (TCP/JPEG)."""

    def __init__(
        self,
        camera_index: int = CAMERA_INDEX,
        server_ip: str = VIDEO_SERVER_IP,
        server_port: int = VIDEO_PORT,
        frame_width: int = FRAME_WIDTH,
        frame_height: int = FRAME_HEIGHT,
        jpeg_quality: int = JPEG_QUALITY,
    ) -> None:
        self.camera_index = camera_index
        self.server_ip = server_ip
        self.server_port = server_port
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.jpeg_quality = jpeg_quality
        self.frame_queue: queue.Queue = queue.Queue(maxsize=2)
        self.stop_event = threading.Event()
        self.camera_thread: Optional[threading.Thread] = None
        self.stream_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self.camera_thread and self.camera_thread.is_alive():
            return
        self.stop_event.clear()
        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.camera_thread.start()
        self.stream_thread.start()
        print(f"[Video] Camera + stream threads started (port {self.server_port})")

    def stop(self) -> None:
        self.stop_event.set()
        if self.camera_thread:
            self.camera_thread.join(timeout=2)
        if self.stream_thread:
            self.stream_thread.join(timeout=2)
        print("[Video] Streams stopped")

    def _camera_loop(self) -> None:
        print(f"[Camera] Opening camera {self.camera_index}...")
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not cap.isOpened():
            print("[Camera] V4L2 failed, fallback to default backend")
            cap.release()
            cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            print(f"[Camera] Error: cannot open camera {self.camera_index}")
            self.stop_event.set()
            return

        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        except cv2.error as exc:
            print(f"[Camera] Failed to set MJPG fourcc: {exc}")

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        cap.set(cv2.CAP_PROP_FPS, 20)

        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if actual_width == 0 or actual_height == 0:
            print(f"[Camera] Invalid resolution {actual_width}x{actual_height}")
            cap.release()
            self.stop_event.set()
            return

        print(f"[Camera] Opened {actual_width}x{actual_height}")

        try:
            while not self.stop_event.is_set():
                ret, frame = cap.read()
                if not ret:
                    print("[Camera] Failed to read frame")
                    time.sleep(0.1)
                    continue
                try:
                    self.frame_queue.put_nowait(frame)
                except queue.Full:
                    pass  # drop frame if queue busy
        finally:
            cap.release()
            print("[Camera] Thread stopped")

    def _stream_loop(self) -> None:
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        while not self.stop_event.is_set():
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.settimeout(1.0)
            try:
                server.bind((self.server_ip, self.server_port))
            except OSError as exc:
                print(f"[Video] Bind error: {exc}. Retrying...")
                time.sleep(1.0)
                server.close()
                continue

            server.listen(1)
            print(f"[Video] Listening on {self.server_ip}:{self.server_port}...")

            try:
                while not self.stop_event.is_set():
                    try:
                        conn, addr = server.accept()
                    except socket.timeout:
                        continue
                    print(f"[Video] Connected: {addr}")
                    self._serve_client(conn, encode_param)
            finally:
                server.close()
                print("[Video] Server socket closed")

    def _serve_client(self, conn: socket.socket, encode_param) -> None:
        with conn:
            frame_count = 0
            start_time = time.time()
            try:
                while not self.stop_event.is_set():
                    try:
                        frame = self.frame_queue.get(timeout=0.1)
                    except queue.Empty:
                        continue

                    success, jpeg = cv2.imencode('.jpg', frame, encode_param)
                    if not success:
                        continue

                    data = jpeg.tobytes()
                    size_bytes = struct.pack('>I', len(data))
                    conn.sendall(size_bytes)
                    conn.sendall(data)

                    frame_count += 1
                    elapsed = time.time() - start_time
                    if elapsed >= 5.0:
                        fps = frame_count / elapsed
                        print(f"[Video] {fps:.1f} FPS | {len(data)/1024:.1f} KB/frame")
                        frame_count = 0
                        start_time = time.time()
            except (BrokenPipeError, ConnectionResetError):
                print("[Video] Quest disconnected")



class QuestVRXLeRobotController:
    """하드웨어 직접 제어에 최적화된 버전"""

    def __init__(self, quest_monitor: QuestSocketMonitor, robot: XLerobot, mirror_mode: bool = False, use_head: bool = True):
        self.quest_monitor = quest_monitor
        self.robot = robot
        self.mirror_mode = mirror_mode
        self.use_head = use_head
        self.kinematics = SO101Kinematics()

        self.arm_state = {
            # Right arm
            "right_arm_shoulder_pan.pos": 0.0,
            "right_arm_shoulder_lift.pos": 0.0,
            "right_arm_elbow_flex.pos": 0.0,
            "right_arm_wrist_flex.pos": 0.0,
            "right_arm_wrist_roll.pos": 0.0,
            "right_arm_gripper.pos": 33.3,  # 2/3 열림 (0=완전열림, 100=완전닫힘)
            # Left arm
            "left_arm_shoulder_pan.pos": 0.0,
            "left_arm_shoulder_lift.pos": 0.0,
            "left_arm_elbow_flex.pos": 0.0,
            "left_arm_wrist_flex.pos": 0.0,
            "left_arm_wrist_roll.pos": 0.0,
            "left_arm_gripper.pos": 33.3,  # 2/3 열림 (0=완전열림, 100=완전닫힘)
        }

        if self.use_head:
            self.arm_state.update({
                "head_motor_1.pos": 0.0,
                "head_motor_2.pos": 0.0,
            })

        # ===== IK 추적용 EE 위치 (meters) - 공식 XLerobot VR 코드와 동일 =====
        # FK 없이 하드코딩 - 초기 자세가 이 EE 위치와 정확히 맞다고 가정
        self.left_ee_pos = np.array([0.1629, 0.1131])   # Quest LEFT → Robot LEFT
        self.right_ee_pos = np.array([0.1629, 0.1131])  # Quest RIGHT → Robot RIGHT

        # ===== Quest VR 추적 변수 =====
        self.prev_left_pos = None
        self.prev_left_euler = {"x": None, "y": None, "z": None}
        self.prev_right_pos = None
        self.prev_right_euler = {"x": None, "y": None, "z": None}
        if self.use_head:
            self.head_offset = {"yaw": None, "pitch": None}  # yaw=y axis, pitch=x axis
            self.head_initial_pos = {"motor1": None, "motor2": None}  # 초기 모터 위치
            self._head_debug_missing = 0
            self._head_debug_uncalibrated = 0
        else:
            self.head_offset = None
            self.head_initial_pos = None

        self.last_left_seen_time = None
        self.last_right_seen_time = None
        self.controller_timeout = 0.5

        # ===== IK 파라미터 (SO101Kinematics에서 관리) =====
        # self.l1, self.l2는 self.kinematics에 있음

        # ===== 제어 스케일 =====
        self.position_scale = 0.5
        self.rotation_scale = 1.0
        self.elbow_ratio = 0.35
        self.wrist_ratio = 0.65
        self.head_pan_scale = 1.0   # Headset yaw → head pan (1:1 비율)
        self.head_tilt_scale = 1.0  # Headset pitch(X) → head tilt (1:1 비율)
        # Quest → 로봇 좌표 매핑 (필요 시 축/부호를 여기서 조정)
        # Oculus Quest: (x=right, y=up, z=forward). 로봇: (x=forward, y=up)
        self.vr_axis_map = {
            "forward": {"index": 2, "sign": 1.0},  # Quest Z → Robot X
            "vertical": {"index": 1, "sign": 1.0},  # Quest Y → Robot Y
        }

        # ===== Gripper 범위 제한 =====
        self.gripper_min = 0.0  # 최대 열림 (2/3 열림)
        self.gripper_max = 60.0  # 완전 닫힘

        # ===== 초기화 =====
        self.is_initialized = False
        self.init_duration = 3.0

    # ========== IK 함수 (SO101Kinematics 사용) ==========
    # SO101Kinematics.inverse_kinematics() 사용
    # FK와 IK가 동일한 좌표 변환을 사용하여 일관성 보장
    # 수학식은 MuJoCo IK와 동일하지만, 좌표 변환 추가로 FK와 매칭

    # ========== Quest 데이터 처리 ==========
    def process_quest_data(self):
        quest_data = self.quest_monitor.get_dual_controllers()
        current_time = time.time()

        # Timeout handling
        if self.last_left_seen_time and current_time - self.last_left_seen_time > self.controller_timeout:
            self.prev_left_pos = None
            self.prev_left_euler = {"x": None, "y": None, "z": None}
            self.last_left_seen_time = None

        if self.last_right_seen_time and current_time - self.last_right_seen_time > self.controller_timeout:
            self.prev_right_pos = None
            self.prev_right_euler = {"x": None, "y": None, "z": None}
            self.last_right_seen_time = None

        if self.mirror_mode:
            left_data = quest_data.get("right")
            right_data = quest_data.get("left")
        else:
            left_data = quest_data.get("left")
            right_data = quest_data.get("right")

        self._process_left_arm(left_data, current_time)
        self._process_right_arm(right_data, current_time)
        if self.use_head:
            self._process_head(quest_data.get("head"))

    def _process_left_arm(self, left_data, current_time):
        """Quest LEFT → Robot LEFT (with IK)"""
        if not left_data or not left_data.get("enabled"):
            return

        pos = left_data["position"]
        euler = left_data.get("euler", {})
        vr_pos = np.array([pos["x"], pos["y"], pos["z"]])

        if self.prev_left_pos is not None:
            # [1] 위치 변화 감지
            delta = (vr_pos - self.prev_left_pos) * self.position_scale
            delta_x, delta_y = self._map_vr_delta(delta, mirror=self.mirror_mode)

            # [2] EE 위치 업데이트
            self.left_ee_pos[0] += delta_x
            self.left_ee_pos[1] += delta_y

            # [3] ⭐ IK 계산 (EE → joint angles) - SO101Kinematics 사용
            shoulder_lift_deg, elbow_flex_deg = self.kinematics.inverse_kinematics(
                self.left_ee_pos[0],
                self.left_ee_pos[1]
            )

            # [4] 회전 delta (shoulder pan)
            if self.prev_left_euler["y"] is not None:
                delta_yaw = self._wrap_delta(euler.get("y", 0) - self.prev_left_euler["y"])
                if self.mirror_mode:
                    delta_yaw *= -1.0
                self.arm_state["left_arm_shoulder_pan.pos"] += delta_yaw * self.rotation_scale
                self.arm_state["left_arm_shoulder_pan.pos"] = np.clip(
                    self.arm_state["left_arm_shoulder_pan.pos"], -120, 120
                )

            # [5] 회전 delta (pitch plane → elbow + wrist)
            pitch_delta = 0.0
            if self.prev_left_euler["x"] is not None:
                delta_pitch = self._wrap_delta(euler.get("x", 0) - self.prev_left_euler["x"])
                pitch_delta = delta_pitch * self.rotation_scale

            # [6] ⭐ 최종 각도 = IK + rotation delta (이미 degrees로 반환됨)
            self.arm_state["left_arm_shoulder_lift.pos"] = shoulder_lift_deg
            self.arm_state["left_arm_elbow_flex.pos"] = elbow_flex_deg + pitch_delta * self.elbow_ratio
            self.arm_state["left_arm_wrist_flex.pos"] += pitch_delta * self.wrist_ratio

            # Clamp (좌표 변환 후 범위)
            # shoulder_lift: 90 - theta1 → 90 - 224.6° ~ 90 - (-45.8°) = -134.6° ~ 135.8°
            # elbow_flex: theta2 - 90 → -51.6° - 90 ~ 220.1° - 90 = -141.6° ~ 130.1°
            self.arm_state["left_arm_shoulder_lift.pos"] = np.clip(self.arm_state["left_arm_shoulder_lift.pos"], -135, 136)
            self.arm_state["left_arm_elbow_flex.pos"] = np.clip(self.arm_state["left_arm_elbow_flex.pos"], -142, 130)
            self.arm_state["left_arm_wrist_flex.pos"] = np.clip(self.arm_state["left_arm_wrist_flex.pos"], -103, 103)

            # [7] 회전 delta (wrist roll)
            if self.prev_left_euler["z"] is not None:
                delta_roll = self._wrap_delta(euler.get("z", 0) - self.prev_left_euler["z"])
                if self.mirror_mode:
                    delta_roll *= -1.0
                self.arm_state["left_arm_wrist_roll.pos"] += delta_roll * self.rotation_scale
                self.arm_state["left_arm_wrist_roll.pos"] = np.clip(
                    self.arm_state["left_arm_wrist_roll.pos"], -180, 180
                )

            # [8] Gripper (범위 제한: 최대 2/3 열림)
            trigger = np.clip(left_data.get("trigger", 0.0), 0.0, 1.0)
            gripper_range = self.gripper_max - self.gripper_min
            self.arm_state["left_arm_gripper.pos"] = (1.0 - trigger) * gripper_range + self.gripper_min

            # Update previous values
            self.prev_left_euler["x"] = euler.get("x", 0)
            self.prev_left_euler["y"] = euler.get("y", 0)
            self.prev_left_euler["z"] = euler.get("z", 0)

        self.prev_left_pos = vr_pos
        self.last_left_seen_time = current_time

    def _process_right_arm(self, right_data, current_time):
        """Quest RIGHT → Robot RIGHT (with IK) - 동일한 로직"""
        if not right_data or not right_data.get("enabled"):
            return

        pos = right_data["position"]
        euler = right_data.get("euler", {})
        vr_pos = np.array([pos["x"], pos["y"], pos["z"]])

        if self.prev_right_pos is not None:
            delta = (vr_pos - self.prev_right_pos) * self.position_scale
            delta_x, delta_y = self._map_vr_delta(delta, mirror=self.mirror_mode)

            self.right_ee_pos[0] += delta_x
            self.right_ee_pos[1] += delta_y

            # ⭐ IK - SO101Kinematics 사용
            shoulder_lift_deg, elbow_flex_deg = self.kinematics.inverse_kinematics(
                self.right_ee_pos[0],
                self.right_ee_pos[1]
            )

            # Shoulder pan
            if self.prev_right_euler["y"] is not None:
                delta_yaw = self._wrap_delta(euler.get("y", 0) - self.prev_right_euler["y"])
                if self.mirror_mode:
                    delta_yaw *= -1.0
                self.arm_state["right_arm_shoulder_pan.pos"] += delta_yaw * self.rotation_scale
                self.arm_state["right_arm_shoulder_pan.pos"] = np.clip(
                    self.arm_state["right_arm_shoulder_pan.pos"], -120, 120
                )

            # Pitch plane
            pitch_delta = 0.0
            if self.prev_right_euler["x"] is not None:
                delta_pitch = self._wrap_delta(euler.get("x", 0) - self.prev_right_euler["x"])
                pitch_delta = delta_pitch * self.rotation_scale

            # ⭐ IK + rotation (이미 degrees로 반환됨)
            self.arm_state["right_arm_shoulder_lift.pos"] = shoulder_lift_deg
            self.arm_state["right_arm_elbow_flex.pos"] = elbow_flex_deg + pitch_delta * self.elbow_ratio
            self.arm_state["right_arm_wrist_flex.pos"] += pitch_delta * self.wrist_ratio

            # Clamp (좌표 변환 후 범위)
            # shoulder_lift: 90 - theta1 → 90 - 224.6° ~ 90 - (-45.8°) = -134.6° ~ 135.8°
            # elbow_flex: theta2 - 90 → -51.6° - 90 ~ 220.1° - 90 = -141.6° ~ 130.1°
            self.arm_state["right_arm_shoulder_lift.pos"] = np.clip(self.arm_state["right_arm_shoulder_lift.pos"], -135, 136)
            self.arm_state["right_arm_elbow_flex.pos"] = np.clip(self.arm_state["right_arm_elbow_flex.pos"], -142, 130)
            self.arm_state["right_arm_wrist_flex.pos"] = np.clip(self.arm_state["right_arm_wrist_flex.pos"], -103, 103)

            # Wrist roll
            if self.prev_right_euler["z"] is not None:
                delta_roll = self._wrap_delta(euler.get("z", 0) - self.prev_right_euler["z"])
                if self.mirror_mode:
                    delta_roll *= -1.0
                self.arm_state["right_arm_wrist_roll.pos"] += delta_roll * self.rotation_scale
                self.arm_state["right_arm_wrist_roll.pos"] = np.clip(
                    self.arm_state["right_arm_wrist_roll.pos"], -180, 180
                )

            # Gripper (범위 제한: 최대 2/3 열림)
            trigger = np.clip(right_data.get("trigger", 0.0), 0.0, 1.0)
            gripper_range = self.gripper_max - self.gripper_min
            self.arm_state["right_arm_gripper.pos"] = (1.0 - trigger) * gripper_range + self.gripper_min

            self.prev_right_euler["x"] = euler.get("x", 0)
            self.prev_right_euler["y"] = euler.get("y", 0)
            self.prev_right_euler["z"] = euler.get("z", 0)

        self.prev_right_pos = vr_pos
        self.last_right_seen_time = current_time

    def _process_head(self, head_data):
        """Headset euler → Head motors (neck_control_full.py 방식)"""
        if not self.use_head:
            return
        if not head_data:
            self._head_debug_missing += 1
            if self._head_debug_missing % 100 == 0:
                print("[HEAD DEBUG] No head data from Quest (check quest_vr app)")
            return

        if (
            self.head_offset["yaw"] is None
            or self.head_offset["pitch"] is None
            or self.head_initial_pos["motor1"] is None
            or self.head_initial_pos["motor2"] is None
        ):
            self._head_debug_uncalibrated += 1
            if self._head_debug_uncalibrated % 100 == 0:
                missing = []
                if self.head_offset["yaw"] is None:
                    missing.append("headset yaw calibration")
                if self.head_offset["pitch"] is None:
                    missing.append("headset pitch calibration")
                if self.head_initial_pos["motor1"] is None or self.head_initial_pos["motor2"] is None:
                    missing.append("robot sync")
                print(f"[HEAD DEBUG] Skipping head control (missing {' & '.join(missing)})")
            return

        euler = head_data.get("euler", {})

        # Headset 좌로 움직이는 것 = +y (yaw, pan)
        yaw = self._normalize_angle(euler.get("y", 0.0))

        # Headset 위로 움직이는 것 = +x (pitch)
        pitch = self._normalize_angle(euler.get("x", 0.0))

        # Delta 계산 (current - calibration_offset)
        delta_yaw = self._wrap_delta(yaw - self.head_offset["yaw"])
        delta_pitch = self._wrap_delta(pitch - self.head_offset["pitch"])

        # 초기 모터 위치 + delta * scale (neck_control_full.py 방식)
        motor1_pos = self.head_initial_pos["motor1"] + (delta_yaw * self.head_pan_scale)
        motor2_pos = self.head_initial_pos["motor2"] + (delta_pitch * self.head_tilt_scale)

        # Clipping
        motor1_pos = np.clip(motor1_pos, -90, 90)
        motor2_pos = np.clip(motor2_pos, -60, 60)

        # DEBUG: 실제 모터 위치도 읽어서 비교
        if not hasattr(self, '_head_debug_counter'):
            self._head_debug_counter = 0
        self._head_debug_counter += 1

        # Debug prints disabled (uncomment to inspect head motor tracking)

        self.arm_state["head_motor_1.pos"] = motor1_pos
        self.arm_state["head_motor_2.pos"] = motor2_pos

    @staticmethod
    def _wrap_delta(delta):
        if delta > 180: delta -= 360
        elif delta < -180: delta += 360
        return delta

    @staticmethod
    def _normalize_angle(angle):
        if angle > 180: angle -= 360
        if angle < -180: angle += 360
        return angle

    def _map_vr_delta(self, delta_vec: np.ndarray, mirror: bool = False) -> tuple[float, float]:
        """
        Quest 컨트롤러 위치 변화(Δx, Δy, Δz)를 로봇 EE 좌표계(Δforward, Δup)로 변환.
        축/부호가 뒤집히면 self.vr_axis_map 값만 바꾸면 됨.
        """
        forward_cfg = self.vr_axis_map["forward"]
        vertical_cfg = self.vr_axis_map["vertical"]
        delta_forward = delta_vec[forward_cfg["index"]] * forward_cfg["sign"]
        delta_vertical = delta_vec[vertical_cfg["index"]] * vertical_cfg["sign"]

        if mirror:
            delta_forward *= -1.0
        return delta_forward, delta_vertical

    # ========== 초기화 & 캘리브레이션 ==========
    def _sync_ee_with_robot(self):
        """arm_state를 실제 로봇 위치와 동기화 (FK 없이, 공식 방식)"""
        obs = self.robot.get_observation()

        # EE 위치는 하드코딩된 값 사용 (공식 XLerobot VR 코드 방식)
        # FK 계산 없이 초기 자세가 [0.1629, 0.1131]과 정확히 맞다고 가정
        print(f"\n[Sync] 하드코딩된 EE 위치 사용:")
        print(f"  Left:  EE=({self.left_ee_pos[0]:.4f}, {self.left_ee_pos[1]:.4f})")
        print(f"  Right: EE=({self.right_ee_pos[0]:.4f}, {self.right_ee_pos[1]:.4f})")

        # arm_state를 실제 로봇 위치로 동기화
        self.arm_state["right_arm_shoulder_pan.pos"] = obs.get("right_arm_shoulder_pan.pos", 0.0)
        self.arm_state["right_arm_shoulder_lift.pos"] = obs.get("right_arm_shoulder_lift.pos", 0.0)
        self.arm_state["right_arm_elbow_flex.pos"] = obs.get("right_arm_elbow_flex.pos", 0.0)
        self.arm_state["right_arm_wrist_flex.pos"] = obs.get("right_arm_wrist_flex.pos", 0.0)
        self.arm_state["right_arm_wrist_roll.pos"] = obs.get("right_arm_wrist_roll.pos", 0.0)
        self.arm_state["left_arm_shoulder_pan.pos"] = obs.get("left_arm_shoulder_pan.pos", 0.0)
        self.arm_state["left_arm_shoulder_lift.pos"] = obs.get("left_arm_shoulder_lift.pos", 0.0)
        self.arm_state["left_arm_elbow_flex.pos"] = obs.get("left_arm_elbow_flex.pos", 0.0)
        self.arm_state["left_arm_wrist_flex.pos"] = obs.get("left_arm_wrist_flex.pos", 0.0)
        self.arm_state["left_arm_wrist_roll.pos"] = obs.get("left_arm_wrist_roll.pos", 0.0)

        # Head motor도 실제 로봇 위치로 동기화 및 초기 위치 저장
        if self.use_head:
            self.arm_state["head_motor_1.pos"] = obs.get("head_motor_1.pos", 0.0)
            self.arm_state["head_motor_2.pos"] = obs.get("head_motor_2.pos", 0.0)
            self.head_initial_pos["motor1"] = self.arm_state["head_motor_1.pos"]
            self.head_initial_pos["motor2"] = self.arm_state["head_motor_2.pos"]

        print(f"  Right joints: pan={self.arm_state['right_arm_shoulder_pan.pos']:.1f}° "
              f"lift={self.arm_state['right_arm_shoulder_lift.pos']:.1f}° "
              f"elbow={self.arm_state['right_arm_elbow_flex.pos']:.1f}°")
        print(f"  Left joints:  pan={self.arm_state['left_arm_shoulder_pan.pos']:.1f}° "
              f"lift={self.arm_state['left_arm_shoulder_lift.pos']:.1f}° "
              f"elbow={self.arm_state['left_arm_elbow_flex.pos']:.1f}°")
        if self.use_head:
            print(f"  Head motors:  motor1={self.arm_state['head_motor_1.pos']:.1f}° "
                  f"motor2={self.arm_state['head_motor_2.pos']:.1f}°")
        print(f"[Sync] arm_state 동기화 완료\n")

    def initialize_smooth(self, loop_hz=50):
        """3초간 초기 자세로 부드럽게 이동"""
        print(f"\n[Initialization] Moving to initial pose over {self.init_duration}s...")
        obs = self.robot.get_observation()

        # [1] 현재 위치 읽기
        current_state = self.arm_state.copy()
        for key in current_state.keys():
            if key in obs:
                current_state[key] = obs[key]

        print("[Step 1] Current pose:")
        print(f"  Right: pan={current_state['right_arm_shoulder_pan.pos']:.1f}° "
              f"lift={current_state['right_arm_shoulder_lift.pos']:.1f}° "
              f"elbow={current_state['right_arm_elbow_flex.pos']:.1f}°")
        print(f"  Left:  pan={current_state['left_arm_shoulder_pan.pos']:.1f}° "
              f"lift={current_state['left_arm_shoulder_lift.pos']:.1f}° "
              f"elbow={current_state['left_arm_elbow_flex.pos']:.1f}°\n")

        # [2] 목표 위치 (초기 자세) - 공식 XLerobot VR 코드와 동일 (zero position)
        head_target1 = 0.0 if not self.is_initialized else current_state.get("head_motor_1.pos", 0.0)
        head_target2 = 0.0 if not self.is_initialized else current_state.get("head_motor_2.pos", 0.0)

        target_state = {
            "right_arm_shoulder_pan.pos": 0.0,
            "right_arm_shoulder_lift.pos": 0.0,
            "right_arm_elbow_flex.pos": 0.0,
            "right_arm_wrist_flex.pos": 0.0,
            "right_arm_wrist_roll.pos": 0.0,
            "right_arm_gripper.pos": 33.3,  # 2/3 열림
            "left_arm_shoulder_pan.pos": 0.0,
            "left_arm_shoulder_lift.pos": 0.0,
            "left_arm_elbow_flex.pos": 0.0,
            "left_arm_wrist_flex.pos": 0.0,
            "left_arm_wrist_roll.pos": 0.0,
            "left_arm_gripper.pos": 33.3,  # 2/3 열림
            "head_motor_1.pos": head_target1,
            "head_motor_2.pos": head_target2,
        }

        print("[Step 2] Target pose:")
        print(f"  Right: pan={target_state['right_arm_shoulder_pan.pos']:.1f}° "
              f"lift={target_state['right_arm_shoulder_lift.pos']:.1f}° "
              f"elbow={target_state['right_arm_elbow_flex.pos']:.1f}°")
        print(f"  Left:  pan={target_state['left_arm_shoulder_pan.pos']:.1f}° "
              f"lift={target_state['left_arm_shoulder_lift.pos']:.1f}° "
              f"elbow={target_state['left_arm_elbow_flex.pos']:.1f}°\n")

        print(f"[Step 3] Starting smooth motion over {self.init_duration}s...\n")

        # [3] 부드럽게 이동
        init_frames = max(1, int(self.init_duration * loop_hz))
        period = 1.0 / loop_hz

        for frame in range(init_frames):
            start = time.time()
            t = (frame + 1) / init_frames  # 0 to 1

            # Linear interpolation
            for key in target_state.keys():
                if key in current_state:
                    self.arm_state[key] = current_state[key] * (1 - t) + target_state[key] * t

            action = dict(self.arm_state)
            self.robot.send_action(action)

            if (frame + 1) % 10 == 0 or frame == init_frames - 1:
                progress = (frame + 1) / init_frames * 100
                print(f"  Progress: {progress:.0f}% ({frame + 1}/{init_frames})")

            dt = time.time() - start
            time.sleep(max(0.0, period - dt))

        # [4] 안정화
        print("Waiting for robot to settle...")
        time.sleep(0.5)

        # [5] EE 위치 동기화
        self._sync_ee_with_robot()
        self.is_initialized = True
        print("✓ Initial pose reached!\n")

    def _calibrate_controller(self, controller_name, duration=2.0, sample_rate=50):
        """컨트롤러 캘리브레이션 (2초간 평균)"""
        print(f"\n[{controller_name}] Hold controller in starting position...")
        print(f"Collecting data for {duration} seconds...")

        samples_pos = []
        samples_euler = {"x": [], "y": [], "z": []}

        start_time = time.time()
        sample_count = 0
        period = 1.0 / sample_rate

        # Check if THIS specific controller has already been calibrated
        if controller_name.lower() == "left" and self.prev_left_pos is not None:
            print(f"  Using cached {controller_name} controller offsets (already calibrated).")
            return True
        elif controller_name.lower() == "right" and self.prev_right_pos is not None:
            print(f"  Using cached {controller_name} controller offsets (already calibrated).")
            return True

        while time.time() - start_time < duration:
            loop_start = time.time()

            quest_data = self.quest_monitor.get_dual_controllers()
            data = quest_data.get(controller_name.lower())

            if data and data.get("enabled"):
                pos = data["position"]
                samples_pos.append([pos["x"], pos["y"], pos["z"]])

                euler = data.get("euler", {})
                samples_euler["x"].append(euler.get("x", 0))
                samples_euler["y"].append(euler.get("y", 0))
                samples_euler["z"].append(euler.get("z", 0))

                sample_count += 1

            elapsed = time.time() - start_time
            progress = (elapsed / duration) * 100
            print(f"  Progress: {progress:.0f}% ({sample_count} samples)", end="\r")

            dt = time.time() - loop_start
            time.sleep(max(0.0, period - dt))

        print()

        if sample_count == 0:
            print(f"  ✗ No data received for {controller_name}!")
            return False

        avg_pos = np.mean(samples_pos, axis=0)
        avg_euler = {
            "x": np.mean(samples_euler["x"]),
            "y": np.mean(samples_euler["y"]),
            "z": np.mean(samples_euler["z"])
        }

        if controller_name.lower() == "left":
            self.prev_left_pos = avg_pos
            self.prev_left_euler = avg_euler.copy()
            self.last_left_seen_time = time.time()
        elif controller_name.lower() == "right":
            self.prev_right_pos = avg_pos
            self.prev_right_euler = avg_euler.copy()
            self.last_right_seen_time = time.time()

        print(f"  ✓ {controller_name} calibrated ({sample_count} samples)")
        print(f"    Pos: ({avg_pos[0]:.3f}, {avg_pos[1]:.3f}, {avg_pos[2]:.3f})")
        print(f"    Euler: (x={avg_euler['x']:.1f}°, y={avg_euler['y']:.1f}°, z={avg_euler['z']:.1f}°)")

        return True

    def _calibrate_headset(self, duration=2.0, sample_rate=50):
        """헤드셋 캘리브레이션 (2초간 평균) - yaw(좌우), tilt(위아래)"""
        if not self.use_head:
            return True
        print(f"\n[Headset] Look forward in neutral position...")
        print(f"Collecting data for {duration} seconds...")

        samples_yaw = []
        samples_pitch = []

        start_time = time.time()
        sample_count = 0
        period = 1.0 / sample_rate

        while time.time() - start_time < duration:
            loop_start = time.time()

            quest_data = self.quest_monitor.get_dual_controllers()
            head_data = quest_data.get("head")

            if head_data:
                euler = head_data.get("euler", {})
                yaw = self._normalize_angle(euler.get("y", 0.0))    # 좌우
                pitch = self._normalize_angle(euler.get("x", 0.0))  # 위아래

                samples_yaw.append(yaw)
                samples_pitch.append(pitch)
                sample_count += 1

            elapsed = time.time() - start_time
            progress = (elapsed / duration) * 100
            print(f"  Progress: {progress:.0f}% ({sample_count} samples)", end="\r")

            dt = time.time() - loop_start
            time.sleep(max(0.0, period - dt))

        print()

        if sample_count == 0:
            print(f"  ✗ No headset data received!")
            return False

        self.head_offset["yaw"] = np.mean(samples_yaw)
        self.head_offset["pitch"] = np.mean(samples_pitch)

        print(f"  ✓ Headset calibrated ({sample_count} samples)")
        print(f"    Yaw: {self.head_offset['yaw']:.1f}°, Pitch: {self.head_offset['pitch']:.1f}°")

        return True

    def run(self, loop_hz=50):
        """메인 루프"""
        period = 1.0 / loop_hz
        print(f"Starting hardware teleop at {loop_hz} Hz...")

        # [1] 초기 자세로 이동 (3초)
        if not self.is_initialized:
            self.initialize_smooth(loop_hz)

        # [2] Quest 장치 캘리브레이션
        print("\n" + "="*60)
        print("Quest VR Device Calibration")
        print("="*60)
        print("\nEach device will be calibrated sequentially.")
        print("Hold each device steady during its 2-second calibration.\n")

        self._calibrate_controller("Left", duration=2.0)
        self._calibrate_controller("Right", duration=2.0)
        if self.use_head:
            self._calibrate_headset(duration=2.0)

        print("\n" + "="*60)
        print("✓ All devices calibrated. Starting teleoperation...")
        print("="*60 + "\n")

        # [3] 텔레오퍼레이션 루프
        try:
            frame_count = 0
            while True:
                start = time.time()

                # [1] Quest 데이터 처리 (IK 계산 포함)
                self.process_quest_data()

                # [2] 로봇에 직접 전송 (dict 변환 불필요!)
                action = dict(self.arm_state)

                # Head 디버깅: action에 head motor가 포함되는지 확인
                if self.use_head and frame_count % 50 == 0:
                    head1 = action.get("head_motor_1.pos", "MISSING")
                    head2 = action.get("head_motor_2.pos", "MISSING")
                    print(f"[ACTION DEBUG] head_motor_1: {head1}, head_motor_2: {head2}")

                # 첫 프레임 디버깅: 텔레오퍼레이션 시작 시 명령 확인
                if frame_count == 0:
                    print(f"\n{'='*60}")
                    print("[첫 프레임] 텔레오퍼레이션 시작 명령")
                    print(f"{'='*60}")
                    print(f"Left arm:  lift={action['left_arm_shoulder_lift.pos']:.2f}° elbow={action['left_arm_elbow_flex.pos']:.2f}°")
                    print(f"Right arm: lift={action['right_arm_shoulder_lift.pos']:.2f}° elbow={action['right_arm_elbow_flex.pos']:.2f}°")
                    print(f"Left EE:  ({self.left_ee_pos[0]:.4f}, {self.left_ee_pos[1]:.4f})")
                    print(f"Right EE: ({self.right_ee_pos[0]:.4f}, {self.right_ee_pos[1]:.4f})")
                    print(f"{'='*60}\n")

                self.robot.send_action(action)

                if frame_count % 50 == 0:
                    left_ok = "✓" if self.prev_left_pos is not None else "✗"
                    right_ok = "✓" if self.prev_right_pos is not None else "✗"
                    print(f"Frame {frame_count}: L{left_ok} R{right_ok}", end="\r")

                frame_count += 1
                dt = time.time() - start
                time.sleep(max(0.0, period - dt))
        except KeyboardInterrupt:
            print("\n\nStopping...")


def main():
    print("=" * 60)
    print("Quest VR XLeRobot Controller (Refactored)")
    print("=" * 60)

    quest_monitor = QuestSocketMonitor(host="0.0.0.0", port=5454)
    quest_monitor.start()

    robot_config = XLerobotConfig(use_degrees=True)
    robot = XLerobotArmOnly(robot_config)
    robot.connect()

    video_streamer = QuestVideoStreamer()
    video_streamer.start()

    if not quest_monitor.wait_for_connection(timeout=60):
        print("Quest connection timeout!")
        quest_monitor.stop()
        robot.disconnect()
        video_streamer.stop()
        return

    controller = QuestVRXLeRobotController(quest_monitor, robot)

    try:
        controller.run(loop_hz=50)
    finally:
        quest_monitor.stop()
        robot.disconnect()
        video_streamer.stop()


if __name__ == "__main__":
    main()
