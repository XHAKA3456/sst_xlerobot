#!/usr/bin/env python3
"""
Quest VR XLeRobot Dataset Recording with Config Support
Quest VR로 로봇을 제어하면서 LeRobot 데이터셋을 수집합니다.

사용법:
    python quest_vr_record_dataset.py --config config.yaml
"""

import os
import sys
import time
import argparse
import json
import yaml
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, List, Optional

import numpy as np
import cv2
import socket
import queue
import threading
from sst_xlerbot.quest.quest_socket_monitor import QuestSocketMonitor
from sst_xlerbot.teleop.quest_vr_xlerobot_controller_no_base import (
    XLerobotArmOnly,
    QuestVRXLeRobotController,
)

try:
    from lerobot.robots.xlerobot import XLerobotConfig
except ModuleNotFoundError:
    REPO_ROOT = Path(__file__).resolve().parents[4]
    LEROBOT_SRC = REPO_ROOT / "lerobot" / "src"
    sys.path.insert(0, str(LEROBOT_SRC))
    from lerobot.robots.xlerobot import XLerobotConfig


def load_config(config_path: str) -> Dict[str, Any]:
    """Load configuration from YAML file"""
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


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
            pass  # 큐가 꽉 차면 프레임 드랍

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

                            # JPEG 인코딩
                            _, buffer = cv2.imencode('.jpg', frame, encode_param)
                            data = buffer.tobytes()

                            # 크기 + 데이터 전송
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

        print("[Quest Stream] Thread stopped")


class CameraManager:
    """다중 카메라 관리"""

    def __init__(self, camera_configs: Dict[str, Dict]):
        self.cameras = {}
        self.camera_names = []

        for name, config in camera_configs.items():
            if not config.get('enabled', True):
                continue

            if config['type'] == 'opencv':
                cap = cv2.VideoCapture(config['index'])
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, config['width'])
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config['height'])
                cap.set(cv2.CAP_PROP_FPS, config['fps'])

                if cap.isOpened():
                    self.cameras[name] = cap
                    self.camera_names.append(name)
                    print(f"✅ Camera '{name}' opened (index {config['index']})")
                else:
                    print(f"⚠️  Failed to open camera '{name}' (index {config['index']})")
            elif config['type'] == 'realsense':
                print(f"⚠️  RealSense camera '{name}' not yet implemented")
                # TODO: Implement RealSense support

        print(f"\nTotal cameras active: {len(self.cameras)}")

    def capture_all(self) -> Dict[str, np.ndarray]:
        """모든 카메라에서 프레임 캡처"""
        frames = {}
        for name, cap in self.cameras.items():
            ret, frame = cap.read()
            if ret:
                frames[name] = frame
        return frames

    def release_all(self):
        """모든 카메라 해제"""
        for cap in self.cameras.values():
            cap.release()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass  # Headless 환경에서는 무시


class DatasetRecorder:
    """LeRobot 형식 데이터셋 레코더 with Hugging Face Hub support"""

    def __init__(
        self,
        config: Dict[str, Any],
        camera_names: List[str],
    ):
        self.config = config
        self.camera_names = camera_names

        dataset_config = config['dataset']
        self.repo_id = dataset_config['repo_id']
        self.task_description = dataset_config['task']
        self.fps = config['recording']['fps']
        self.push_to_hub = dataset_config.get('push_to_hub', False)
        self.resume = dataset_config.get('resume', False)

        # 데이터셋 디렉토리 생성
        output_dir = Path(dataset_config['output_dir'])
        self.dataset_dir = output_dir / self.repo_id.replace("/", "_")

        # Resume 기능: 기존 데이터셋 확인
        if self.resume and self.dataset_dir.exists():
            self.episode_index = self._find_last_episode_index() + 1
            print(f"📂 Resuming from episode {self.episode_index}")
        else:
            self.episode_index = 0
            self.dataset_dir.mkdir(parents=True, exist_ok=True)

        self.frame_index = 0
        self.episode_data = []

        # Video writer
        self.video_writer = None
        self.save_video = config['visualization'].get('save_video', True)
        self.video_codec = config['visualization'].get('video_codec', 'mp4v')

        # Display settings
        self.display_cameras = config['visualization'].get('display_cameras', False)

        print(f"\n{'='*60}")
        print(f"Dataset Directory: {self.dataset_dir}")
        if self.push_to_hub:
            print(f"Will push to Hub: https://huggingface.co/datasets/{self.repo_id}")
        print(f"{'='*60}\n")

    def start_episode(self):
        """에피소드 시작"""
        self.frame_index = 0
        self.episode_data = []

        # Video writer 초기화
        if self.save_video:
            episode_dir = self.dataset_dir / f"episode_{self.episode_index:04d}"
            episode_dir.mkdir(parents=True, exist_ok=True)

            video_path = episode_dir / "episode_video.mp4"
            fourcc = cv2.VideoWriter_fourcc(*self.video_codec)
            # Use first camera resolution for video
            self.video_writer = cv2.VideoWriter(
                str(video_path), fourcc, self.fps, (640, 480)
            )

        print(f"\n{'='*60}")
        print(f"Episode {self.episode_index} Started")
        print(f"{'='*60}\n")

    def record_frame(
        self,
        observation: dict,
        action: dict,
        camera_frames: Dict[str, np.ndarray]
    ):
        """프레임 기록"""
        # State 벡터 추출 (관절 위치)
        state = self._extract_state(observation)
        action_vec = self._extract_action(action)

        # 프레임 데이터 저장
        frame_data = {
            "episode_index": self.episode_index,
            "frame_index": self.frame_index,
            "timestamp": time.time(),
            "observation": {
                "state": state.tolist(),
            },
            "action": action_vec.tolist(),
        }

        # 카메라 이미지 저장
        for cam_name, frame in camera_frames.items():
            img_dir = self.dataset_dir / f"episode_{self.episode_index:04d}" / "images" / cam_name
            img_dir.mkdir(parents=True, exist_ok=True)
            img_path = img_dir / f"frame_{self.frame_index:06d}.png"
            cv2.imwrite(str(img_path), frame)
            frame_data["observation"][f"image_{cam_name}"] = str(img_path.relative_to(self.dataset_dir))

            # Video에 첫 번째 카메라 프레임 추가
            if self.save_video and self.video_writer is not None and cam_name == self.camera_names[0]:
                # Resize if needed
                resized = cv2.resize(frame, (640, 480))
                self.video_writer.write(resized)

        # Display cameras on screen
        if self.display_cameras:
            self._display_camera_frames(camera_frames)

        self.episode_data.append(frame_data)
        self.frame_index += 1

    def _display_camera_frames(self, camera_frames: Dict[str, np.ndarray]):
        """카메라 프레임들을 가로로 이어붙여서 하나의 창에 표시"""
        if not camera_frames:
            return

        try:
            # 모든 프레임을 640x480으로 리사이즈
            resized_frames = []
            for cam_name in self.camera_names:
                if cam_name in camera_frames:
                    frame = camera_frames[cam_name]
                    resized = cv2.resize(frame, (640, 480))

                    # 카메라 이름을 프레임에 표시
                    cv2.putText(
                        resized,
                        cam_name,
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2
                    )

                    resized_frames.append(resized)

            # 가로로 이어붙이기
            if resized_frames:
                combined = np.hstack(resized_frames)
                cv2.imshow('Dataset Recording - Camera Feeds', combined)
                cv2.waitKey(1)  # 1ms 대기 (화면 업데이트용)
        except cv2.error as e:
            # OpenCV GUI 지원 안 되면 스킵 (headless 환경)
            if "not implemented" in str(e).lower():
                # 첫 번째 에러만 출력
                if not hasattr(self, '_display_error_shown'):
                    print("⚠️  OpenCV GUI not available (running headless). Skipping display.")
                    self._display_error_shown = True
                # display_cameras 자동 비활성화
                self.display_cameras = False
            else:
                raise

    def end_episode(self):
        """에피소드 종료 및 저장"""
        # 에피소드 데이터를 JSON으로 저장
        episode_dir = self.dataset_dir / f"episode_{self.episode_index:04d}"
        episode_dir.mkdir(parents=True, exist_ok=True)

        episode_file = episode_dir / "episode_data.json"
        with open(episode_file, 'w') as f:
            json.dump({
                "episode_index": self.episode_index,
                "num_frames": self.frame_index,
                "task": self.task_description,
                "fps": self.fps,
                "frames": self.episode_data,
            }, f, indent=2)

        # Video writer 종료
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None

        print(f"\n{'='*60}")
        print(f"Episode {self.episode_index} Completed")
        print(f"Total frames: {self.frame_index}")
        print(f"Saved to: {episode_file}")
        print(f"{'='*60}\n")

        self.episode_index += 1
        self.episode_data = []

    def _extract_state(self, observation: dict) -> np.ndarray:
        """관측에서 state 벡터 추출"""
        state_keys = [
            "left_arm_shoulder_pan.pos",
            "left_arm_shoulder_lift.pos",
            "left_arm_elbow_flex.pos",
            "left_arm_wrist_flex.pos",
            "left_arm_wrist_roll.pos",
            "left_arm_gripper.pos",
            "right_arm_shoulder_pan.pos",
            "right_arm_shoulder_lift.pos",
            "right_arm_elbow_flex.pos",
            "right_arm_wrist_flex.pos",
            "right_arm_wrist_roll.pos",
            "right_arm_gripper.pos",
            "head_motor_1.pos",
            "head_motor_2.pos",
        ]

        state = []
        for key in state_keys:
            state.append(observation.get(key, 0.0))

        return np.array(state, dtype=np.float32)

    def _extract_action(self, action: dict) -> np.ndarray:
        """액션 딕셔너리를 벡터로 변환"""
        action_keys = [
            "left_arm_shoulder_pan.pos",
            "left_arm_shoulder_lift.pos",
            "left_arm_elbow_flex.pos",
            "left_arm_wrist_flex.pos",
            "left_arm_wrist_roll.pos",
            "left_arm_gripper.pos",
            "right_arm_shoulder_pan.pos",
            "right_arm_shoulder_lift.pos",
            "right_arm_elbow_flex.pos",
            "right_arm_wrist_flex.pos",
            "right_arm_wrist_roll.pos",
            "right_arm_gripper.pos",
            "head_motor_1.pos",
            "head_motor_2.pos",
        ]

        action_vec = []
        for key in action_keys:
            action_vec.append(action.get(key, 0.0))

        return np.array(action_vec, dtype=np.float32)

    def _find_last_episode_index(self) -> int:
        """Find the last episode index in existing dataset"""
        episode_dirs = list(self.dataset_dir.glob("episode_*"))
        if not episode_dirs:
            return -1

        indices = []
        for ep_dir in episode_dirs:
            try:
                idx = int(ep_dir.name.split("_")[1])
                indices.append(idx)
            except (IndexError, ValueError):
                continue

        return max(indices) if indices else -1

    def save_metadata(self):
        """데이터셋 메타데이터 저장"""
        metadata = {
            "repo_id": self.repo_id,
            "task": self.task_description,
            "fps": self.fps,
            "total_episodes": self.episode_index,
            "created_at": datetime.now().isoformat(),
            "robot_type": "xlerobot",
            "control_method": "quest_vr",
            "cameras": self.camera_names,
        }

        metadata_file = self.dataset_dir / "metadata.json"
        with open(metadata_file, 'w') as f:
            json.dump(metadata, f, indent=2)

        print(f"\n✅ Metadata saved to: {metadata_file}")

    def push_to_huggingface_hub(self, hub_token: Optional[str] = None):
        """Hugging Face Hub에 데이터셋 업로드"""
        if not self.push_to_hub:
            return

        # Get token from environment or parameter
        token = hub_token or os.environ.get('HF_TOKEN')
        if not token:
            print("⚠️  No Hugging Face token provided!")
            print("   Set HF_TOKEN environment variable or login with 'huggingface-cli login'")
            return

        print(f"\n{'='*60}")
        print("Pushing dataset to Hugging Face Hub...")
        print(f"{'='*60}\n")

        try:
            from huggingface_hub import HfApi, create_repo

            api = HfApi(token=token)

            # Create repository if it doesn't exist
            try:
                create_repo(
                    repo_id=self.repo_id,
                    repo_type="dataset",
                    private=False,
                    token=token,
                    exist_ok=True,
                )
                print(f"✅ Repository created/verified: {self.repo_id}")
            except Exception as e:
                print(f"⚠️  Repository creation: {e}")

            # Upload dataset folder
            print(f"Uploading files from {self.dataset_dir}...")
            api.upload_folder(
                folder_path=str(self.dataset_dir),
                repo_id=self.repo_id,
                repo_type="dataset",
                token=token,
            )

            print(f"\n✅ Dataset successfully pushed to:")
            print(f"   https://huggingface.co/datasets/{self.repo_id}")

        except ImportError:
            print("⚠️  huggingface_hub not installed. Install with:")
            print("   pip install huggingface_hub")
        except Exception as e:
            print(f"❌ Error pushing to hub: {e}")


def main():
    parser = argparse.ArgumentParser(description="Quest VR Dataset Recording with Config")
    parser.add_argument("--config", type=str, required=True,
                        help="Path to config YAML file")
    args = parser.parse_args()

    # Load configuration
    print("=" * 60)
    print("Quest VR XLeRobot Dataset Recording")
    print("=" * 60)
    print(f"\nLoading config from: {args.config}")

    config = load_config(args.config)

    print(f"Dataset: {config['dataset']['repo_id']}")
    print(f"Task: {config['dataset']['task']}")
    print(f"Episodes: {config['recording']['num_episodes']}")
    print(f"Resume: {config['dataset'].get('resume', False)}")
    print()

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
    print(f"\n[DEBUG] stream_camera_name: {stream_camera_name}")
    print(f"[DEBUG] Available cameras: {camera_manager.camera_names}")

    if stream_camera_name:
        if stream_camera_name in camera_manager.camera_names:
            print(f"✅ Quest streaming from '{stream_camera_name}'")
            quest_streamer = QuestFrameStreamer(
                server_ip=quest_config.get('host', '0.0.0.0'),
                server_port=quest_config.get('video_port', 5656)
            )
            quest_streamer.start()
        else:
            print(f"⚠️  Camera '{stream_camera_name}' not found or disabled")
            print(f"    Available: {camera_manager.camera_names}")
    else:
        print("⚠️  No camera specified for Quest streaming")

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
    controller = QuestVRXLeRobotController(quest_monitor, robot, use_head=use_head)

    # 데이터셋 레코더 생성
    recorder = DatasetRecorder(
        config=config,
        camera_names=camera_manager.camera_names,
    )

    try:
        # 초기화 (3초간 초기 자세로 이동)
        print("\n[1/2] Initializing robot...")
        controller.initialize_smooth(loop_hz=config['recording']['fps'])

        # 캘리브레이션
        print("\n[2/2] Calibrating Quest VR devices...")
        print("="*60)
        controller._calibrate_controller("Left", duration=2.0)
        controller._calibrate_controller("Right", duration=2.0)
        controller._calibrate_headset(duration=2.0)
        print("="*60)

        print("\n✅ Ready to record!\n")

        # 에피소드 루프
        num_episodes = config['recording']['num_episodes']
        episode_time = config['recording']['episode_time']
        fps = config['recording']['fps']

        for episode_idx in range(num_episodes):
            print(f"\n{'='*60}")
            print(f"Episode {episode_idx + 1}/{num_episodes}")
            print(f"{'='*60}")
            print("\n준비되면 'r' 입력 후 Enter를 눌러주세요...")
            while True:
                user_input = input().strip().lower()
                if user_input == 'r':
                    break
                elif user_input == 'q':
                    print("Recording cancelled by user")
                    raise KeyboardInterrupt

            recorder.start_episode()

            print(f"\n🔴 Recording for {episode_time} seconds...")
            print("Ctrl+C를 눌러 에피소드를 일찍 종료할 수 있습니다.\n")

            start_time = time.time()
            period = 1.0 / fps

            try:
                # 에피소드 기록 루프
                while time.time() - start_time < episode_time:
                    loop_start = time.time()

                    # Quest 데이터 처리 및 로봇 제어
                    controller.process_quest_data()
                    action = dict(controller.arm_state)
                    robot.send_action(action)

                    # 관측 읽기
                    observation = robot.get_observation()

                    # 카메라 프레임 캡처
                    camera_frames = camera_manager.capture_all()

                    # Quest로 스트리밍 (stream_camera가 있으면)
                    if quest_streamer and stream_camera_name in camera_frames:
                        quest_streamer.send_frame(camera_frames[stream_camera_name])
                        # 디버그: 프레임 전송 확인
                        if recorder.frame_index % 100 == 0:
                            print(f"[DEBUG] Sent frame to Quest: {stream_camera_name}")

                    # 프레임 기록
                    recorder.record_frame(observation, action, camera_frames)

                    # 진행 상황 출력
                    if recorder.frame_index % 30 == 0:
                        elapsed = time.time() - start_time
                        remaining = episode_time - elapsed
                        print(f"Frame {recorder.frame_index:04d} | "
                              f"Time: {elapsed:5.1f}s / {episode_time}s | "
                              f"Remaining: {remaining:5.1f}s", end="\r")

                    # FPS 유지
                    dt = time.time() - loop_start
                    time.sleep(max(0.0, period - dt))

            except KeyboardInterrupt:
                print("\n\n⏸️  Episode ended by user")

            recorder.end_episode()

            # 다음 에피소드 준비
            if episode_idx < num_episodes - 1:
                reset_time = config['recording'].get('reset_time', 10)

                print(f"\n{'='*60}")
                print(f"Reset Time: {reset_time} seconds")
                print(f"{'='*60}")
                print("환경을 리셋하고 다음 에피소드를 준비하세요.\n")

                # 카운트다운
                for remaining in range(reset_time, 0, -1):
                    print(f"⏱️  다음 에피소드까지: {remaining}초 남음...", end="\r")
                    time.sleep(1)

                print("\n\n준비되면 'r' 입력 후 Enter (종료하려면 'q')...")

        # 메타데이터 저장
        recorder.save_metadata()

        # Hugging Face Hub에 업로드
        if config['dataset'].get('push_to_hub'):
            print("\n" + "="*60)
            print("Hugging Face Hub Upload")
            print("="*60)
            hub_token = os.environ.get('HF_TOKEN')
            if not hub_token:
                print("Enter your Hugging Face token (or press Enter to skip):")
                hub_token = input().strip() or None

            if hub_token:
                recorder.push_to_huggingface_hub(hub_token)
            else:
                print("⚠️  Skipping Hub upload (no token provided)")

        print(f"\n{'='*60}")
        print("✅ Recording Complete!")
        print(f"{'='*60}")
        print(f"\nDataset: {recorder.dataset_dir}")
        print(f"Total episodes: {recorder.episode_index}")
        print(f"Task: {config['dataset']['task']}\n")

    except KeyboardInterrupt:
        print("\n\n⏹️  Recording stopped by user")
        recorder.save_metadata()
        if config['dataset'].get('push_to_hub'):
            print("\nDo you want to push the partial dataset to Hub? (y/n): ", end="")
            if input().strip().lower() == 'y':
                hub_token = os.environ.get('HF_TOKEN')
                if not hub_token:
                    print("Enter your Hugging Face token:")
                    hub_token = input().strip() or None
                recorder.push_to_huggingface_hub(hub_token)

    finally:
        quest_monitor.stop()
        robot.disconnect()
        camera_manager.release_all()
        if quest_streamer:
            quest_streamer.stop()
        try:
            cv2.destroyAllWindows()  # Close all OpenCV windows
        except cv2.error:
            pass  # Headless 환경
        print("\n✅ All resources released safely")


if __name__ == "__main__":
    main()
