#!/usr/bin/env python3
from __future__ import annotations
"""
Quest VR XLeRobot Dataset Recording - LeRobot 규격 준수 버전
Quest VR로 로봇을 제어하면서 LeRobot 데이터셋을 수집합니다.

사용법:
    python quest_vr_record_dataset_v2.py --config config.yaml
"""

import os
import sys
import time
import argparse
import yaml
import select
from pathlib import Path
from typing import Dict, Any

import numpy as np
import cv2
import socket
import queue
import threading
import rerun as rr

from sst_xlerbot.quest.quest_socket_monitor import QuestSocketMonitor
from sst_xlerbot.teleop.quest_vr_xlerobot_controller_no_base import (
    XLerobotArmOnly,
    QuestVRXLeRobotController,
)

try:
    from lerobot.robots.xlerobot import XLerobotConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.datasets.utils import hw_to_dataset_features, build_dataset_frame
    from lerobot.utils.constants import ACTION, OBS_STR
    from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
except ModuleNotFoundError:
    REPO_ROOT = Path(__file__).resolve().parents[4]
    LEROBOT_SRC = REPO_ROOT / "lerobot" / "src"
    sys.path.insert(0, str(LEROBOT_SRC))
    from lerobot.robots.xlerobot import XLerobotConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.datasets.utils import hw_to_dataset_features, build_dataset_frame
    from lerobot.utils.constants import ACTION, OBS_STR
    from lerobot.utils.visualization_utils import init_rerun, log_rerun_data


def load_config(config_path: str) -> Dict[str, Any]:
    """Load configuration from YAML file"""
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def poll_for_quit(timeout: float = 0.0) -> bool:
    """Check stdin for 'q' + Enter without blocking."""
    try:
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
    except (OSError, ValueError):
        return False

    if not ready:
        return False

    user_input = sys.stdin.readline()
    if not user_input:
        return False

    return user_input.strip().lower() == "q"


def build_dataset_features(robot, camera_manager: "CameraManager", use_videos: bool = True) -> Dict[str, Dict]:
    """Construct LeRobot dataset feature spec from robot + external cameras."""
    # Remove base velocity commands since the omnidirectional base is not used.
    def _should_keep(key: str) -> bool:
        if key.endswith(".vel") or key.startswith("base_"):
            return False
        if not getattr(robot, "use_head", True) and "head_motor" in key:
            return False
        return True

    filtered_action_hw = {key: spec for key, spec in robot.action_features.items() if _should_keep(key)}

    filtered_obs_hw = {key: spec for key, spec in robot.observation_features.items() if _should_keep(key)}

    features = {}
    features.update(hw_to_dataset_features(filtered_action_hw, ACTION))
    features.update(hw_to_dataset_features(filtered_obs_hw, OBS_STR, use_videos))

    for name in camera_manager.dataset_camera_names:
        cam_cfg = camera_manager.camera_configs.get(name, {})
        height = cam_cfg.get('height', 480)
        width = cam_cfg.get('width', 640)
        features[f"{OBS_STR}.images.{name}"] = {
            "dtype": "video" if use_videos else "image",
            "shape": (height, width, 3),
            "names": ["height", "width", "channels"],
        }

    return features


def prepare_dataset(
    dataset_cfg: Dict[str, Any],
    recording_cfg: Dict[str, Any],
    robot,
    camera_manager: CameraManager,
) -> tuple[LeRobotDataset, Path]:
    """Create or resume a LeRobot dataset according to config."""
    repo_id = dataset_cfg['repo_id']
    fps = recording_cfg['fps']
    default_root = Path(__file__).resolve().parents[4] / "dataset"
    local_root_base = (
        Path(dataset_cfg['output_dir']).expanduser()
        if dataset_cfg.get('output_dir')
        else default_root
    )
    local_root = local_root_base / repo_id.replace("/", "_")
    local_root_parent = local_root.parent
    local_root_parent.mkdir(parents=True, exist_ok=True)

    resume = dataset_cfg.get('resume', False)
    dataset = None

    if resume and local_root.exists():
        dataset = LeRobotDataset(repo_id, root=local_root)
        print(f"📂 Resuming dataset at {local_root} (episodes={dataset.meta.total_episodes})")
    else:
        if local_root.exists() and not resume:
            raise FileExistsError(
                f"Dataset directory {local_root} already exists. Delete it or set dataset.resume=true."
            )
        feature_spec = build_dataset_features(robot, camera_manager, use_videos=True)
        dataset = LeRobotDataset.create(
            repo_id=repo_id,
            fps=fps,
            features=feature_spec,
            robot_type=robot.name,
            root=local_root,
            use_videos=True,
        )
        print(f"📁 Created new dataset structure at {local_root}")

    return dataset, local_root


class QuestFrameStreamer:
    """외부 프레임을 Quest로 스트리밍하는 클래스"""

    def __init__(self, server_ip: str = "0.0.0.0", server_port: int = 5656, jpeg_quality: int = 80):
        self.server_ip = server_ip
        self.server_port = server_port
        self.jpeg_quality = jpeg_quality
        self.frame_queue = queue.Queue(maxsize=2)
        self.stop_event = threading.Event()
        self.stream_thread = None

    def start(self):
        """스트리밍 스레드 시작"""
        if self.stream_thread and self.stream_thread.is_alive():
            return
        self.stop_event.clear()
        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.stream_thread.start()
        print(f"[Quest Stream] Started on {self.server_ip}:{self.server_port}")

    def stop(self):
        """스트리밍 중지"""
        self.stop_event.set()
        if self.stream_thread:
            self.stream_thread.join(timeout=2)
        print("[Quest Stream] Stopped")

    def send_frame(self, frame: np.ndarray):
        """프레임을 Quest로 전송"""
        try:
            self.frame_queue.put_nowait(frame)
        except queue.Full:
            pass

    def _stream_loop(self):
        """TCP 소켓으로 프레임 스트리밍"""
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]

        while not self.stop_event.is_set():
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.settimeout(1.0)

            try:
                server.bind((self.server_ip, self.server_port))
            except OSError as exc:
                print(f"[Quest Stream] Bind error: {exc}")
                time.sleep(1.0)
                server.close()
                continue

            server.listen(1)
            print(f"[Quest Stream] Listening on {self.server_ip}:{self.server_port}...")

            try:
                while not self.stop_event.is_set():
                    try:
                        conn, addr = server.accept()
                        print(f"[Quest Stream] Client connected from {addr}")
                        conn.settimeout(5.0)

                        while not self.stop_event.is_set():
                            try:
                                frame = self.frame_queue.get(timeout=0.1)
                            except queue.Empty:
                                continue

                            bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                            _, buffer = cv2.imencode('.jpg', bgr_frame, encode_param)
                            data = buffer.tobytes()
                            size_bytes = len(data).to_bytes(4, 'big')

                            try:
                                conn.sendall(size_bytes + data)
                            except (BrokenPipeError, ConnectionResetError):
                                print("[Quest Stream] Client disconnected")
                                break

                        conn.close()

                    except socket.timeout:
                        continue
                    except Exception as e:
                        print(f"[Quest Stream] Accept error: {e}")
                        time.sleep(0.5)

            finally:
                server.close()


class CameraManager:
    """다중 카메라 관리"""

    def __init__(self, camera_configs: Dict[str, Dict]):
        self.cameras = {}
        self.camera_names = []  # 모든 카메라 (스트리밍용 포함)
        self.dataset_camera_names = []  # 데이터셋에 저장할 카메라
        self.quest_only_cameras = []
        self.camera_configs = {}

        for name, config in camera_configs.items():
            if not config.get('enabled', True):
                continue

            if config['type'] == 'opencv':
                cap = cv2.VideoCapture(config['index'], cv2.CAP_V4L2)
                if not cap.isOpened():
                    cap.release()
                    cap = cv2.VideoCapture(config['index'])

                if not cap.isOpened():
                    print(f"⚠️  Failed to open camera '{name}' (index {config['index']})")
                    continue

                fourcc = config.get('fourcc', 'MJPG')
                if fourcc:
                    try:
                        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
                    except cv2.error as exc:
                        print(f"⚠️  Failed to set FOURCC {fourcc} on '{name}': {exc}")

                cap.set(cv2.CAP_PROP_FRAME_WIDTH, config['width'])
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config['height'])
                cap.set(cv2.CAP_PROP_FPS, config['fps'])

                self.cameras[name] = cap
                self.camera_names.append(name)
                self.camera_configs[name] = config
                if config.get('quest_only', False):
                    self.quest_only_cameras.append(name)
                else:
                    self.dataset_camera_names.append(name)
                print(f"✅ Camera '{name}' opened (index {config['index']}), fourcc={fourcc}")

        print(f"\nTotal cameras active: {len(self.cameras)}")

    def capture_all(self) -> Dict[str, np.ndarray]:
        """모든 카메라에서 프레임 캡처"""
        frames = {}
        for name, cap in self.cameras.items():
            ret, frame = cap.read()
            if ret:
                frames[name] = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return frames

    def release_all(self):
        """모든 카메라 해제"""
        for cap in self.cameras.values():
            cap.release()


def main():
    parser = argparse.ArgumentParser(description="Quest VR Dataset Recording - LeRobot Format")
    parser.add_argument("--config", type=str, required=True, help="Path to config YAML file")
    args = parser.parse_args()

    # Load configuration
    print("=" * 60)
    print("Quest VR XLeRobot Dataset Recording (LeRobot Format)")
    print("=" * 60)
    print(f"\nLoading config from: {args.config}")

    config = load_config(args.config)

    dataset_config = config['dataset']
    recording_config = config['recording']

    print(f"Dataset: {dataset_config['repo_id']}")
    print(f"Task: {dataset_config['task']}")
    print(f"Episodes: {recording_config['num_episodes']}")
    print(f"Resume: {dataset_config.get('resume', False)}")
    print()

    # Initialize Rerun for visualization
    if config['visualization'].get('display_cameras', False):
        init_rerun(session_name="quest_vr_recording")
        print("✅ Rerun visualization started")

    # 카메라 초기화
    print("Initializing cameras...")
    camera_manager = CameraManager(config['cameras'])

    if len(camera_manager.cameras) == 0:
        print("❌ No cameras available!")
        return

    # Quest 모니터 시작
    quest_config = config['quest']
    quest_monitor = QuestSocketMonitor(
        host=quest_config.get('host', '0.0.0.0'),
        port=quest_config['data_port']
    )
    quest_monitor.start()

    # Quest 비디오 스트리머 시작
    stream_camera_name = quest_config.get('stream_camera')
    quest_streamer = None

    if stream_camera_name and stream_camera_name in camera_manager.camera_names:
        print(f"✅ Quest streaming from '{stream_camera_name}'")
        quest_streamer = QuestFrameStreamer(
            server_ip=quest_config.get('host', '0.0.0.0'),
            server_port=quest_config.get('video_port', 5656)
        )
        quest_streamer.start()

    # 로봇 연결
    robot_config_dict = config['robot']
    use_head = robot_config_dict.get('use_head', True)

    robot_config = XLerobotConfig(
        use_degrees=robot_config_dict.get('use_degrees', True),
        port1=robot_config_dict['port1'],
        port2=robot_config_dict['port2'],
    )
    robot = XLerobotArmOnly(robot_config, use_head=use_head)
    robot.connect()

    # Quest 연결 대기
    print("\nWaiting for Quest VR connection...")
    if not quest_monitor.wait_for_connection(timeout=60):
        print("❌ Quest connection timeout!")
        quest_monitor.stop()
        robot.disconnect()
        camera_manager.release_all()
        return

    # Quest VR 컨트롤러 초기화
    controller = QuestVRXLeRobotController(
        quest_monitor,
        robot,
        mirror_mode=quest_config.get('mirror_mode', False),
        use_head=use_head,
    )

    # LeRobot Dataset 준비
    dataset = None
    dataset_root = None
    push_to_hub_flag = dataset_config.get('push_to_hub', False)
    try:
        dataset, dataset_root = prepare_dataset(dataset_config, recording_config, robot, camera_manager)
    except FileExistsError as exc:
        print(f"❌ {exc}")
        quest_monitor.stop()
        robot.disconnect()
        camera_manager.release_all()
        if quest_streamer:
            quest_streamer.stop()
        return

    print(f"\nDataset root: {dataset_root}")
    print("Ready to record frames directly into LeRobot format.\n")

    episodes_recorded = 0

    try:
        # 초기화
        print("[1/2] Initializing robot...")
        controller.initialize_smooth(loop_hz=recording_config['fps'])

        # 캘리브레이션
        print("\n[2/2] Calibrating Quest VR devices...")
        print("="*60)
        controller._calibrate_controller("Left", duration=2.0)
        controller._calibrate_controller("Right", duration=2.0)
        controller._calibrate_headset(duration=2.0)
        print("="*60)

        print("\n✅ Ready to record episodes!")

        num_episodes = recording_config['num_episodes']
        episode_time = recording_config['episode_time']
        fps = recording_config['fps']
        period = 1.0 / fps
        task_description = dataset_config['task']
        stream_camera_name = quest_config.get('stream_camera')

        for ep_idx in range(num_episodes):
            print(f"\n{'='*60}")
            print(f"Episode {ep_idx + 1}/{num_episodes}")
            print(f"{'='*60}")
            print("Automatically starting episode (press Ctrl+C anytime to abort).")

            episode_started = False
            frame_counter = 0
            start_time = time.time()

            print(f"\n🔴 Recording up to {episode_time} seconds...")
            try:
                while time.time() - start_time < episode_time:
                    loop_start = time.time()
                    controller.process_quest_data()
                    action = dict(controller.arm_state)
                    robot.send_action(action)

                    observation = robot.get_observation()
                    camera_frames = camera_manager.capture_all()

                    if quest_streamer and stream_camera_name in camera_frames:
                        quest_streamer.send_frame(camera_frames[stream_camera_name])

                    # Ensure all configured cameras produced a frame
                    missing_frames = [
                        name for name in camera_manager.dataset_camera_names if name not in camera_frames
                    ]
                    if missing_frames:
                        continue

                    observation_values = dict(observation)
                    for name, frame in camera_frames.items():
                        if name not in camera_manager.dataset_camera_names:
                            continue
                        observation_values[name] = frame

                    try:
                        obs_frame = build_dataset_frame(dataset.features, observation_values, prefix=OBS_STR)
                        act_frame = build_dataset_frame(dataset.features, action, prefix=ACTION)
                    except KeyError as exc:
                        print(f"[WARN] Skipping frame due to missing key: {exc}")
                        continue

                    frame_payload = {
                        **obs_frame,
                        **act_frame,
                        "task": task_description,
                    }
                    dataset_timestamp = time.time()
                    dataset.add_frame(frame_payload)
                    episode_started = True
                    frame_counter += 1

                    if config['visualization'].get('display_cameras', False):
                        log_rerun_data(observation=observation_values, action=action)

                    if frame_counter % fps == 0:
                        elapsed = time.time() - start_time
                        print(
                            f"Frames: {frame_counter:05d} | "
                            f"Elapsed: {elapsed:5.1f}s / {episode_time}s",
                            end="\r",
                        )

                    dt = time.time() - loop_start
                    if period - dt > 0:
                        time.sleep(period - dt)

            except KeyboardInterrupt:
                print("\n⏹️  Episode interrupted by user.")

            if episode_started and dataset.episode_buffer is not None and dataset.episode_buffer["size"] > 0:
                dataset.save_episode()
                episodes_recorded += 1
                print(f"\n✅ Episode {episodes_recorded} saved ({frame_counter} frames).")
            else:
                if dataset.episode_buffer is not None:
                    dataset.clear_episode_buffer()
                print("\n⚠️  Episode skipped (no frames recorded).")

            if ep_idx < num_episodes - 1:
                reset_time = recording_config.get('reset_time', 5)
                if reset_time > 0:
                    print(f"\nReset environment ({reset_time} seconds)... (press 'q' + Enter to stop)")
                    for remaining in range(reset_time, 0, -1):
                        print(f"⏱️  Next episode in {remaining} s", end="\r")
                        if poll_for_quit(1.0):
                            raise KeyboardInterrupt
                    print("\n")

        print(f"\n🎉 Recorded {episodes_recorded} episodes successfully.")

    except KeyboardInterrupt:
        print("\n⏹️  Recording stopped by user.")
        if dataset and dataset.episode_buffer is not None and dataset.episode_buffer["size"] > 0:
            print("Saving partial episode before exiting...")
            dataset.save_episode()
            episodes_recorded += 1
    finally:
        if dataset:
            dataset.finalize()
            if push_to_hub_flag:
                try:
                    print("\nUploading dataset to Hugging Face Hub...")
                    dataset.push_to_hub(private=False)
                    print("✅ Upload complete.")
                except Exception as exc:
                    print(f"⚠️  Failed to push dataset to Hub: {exc}")
        quest_monitor.stop()
        robot.disconnect()
        camera_manager.release_all()
        if quest_streamer:
            quest_streamer.stop()
        print("\n✅ All resources released safely")


if __name__ == "__main__":
    main()
