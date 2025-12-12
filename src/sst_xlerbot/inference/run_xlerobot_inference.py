#!/usr/bin/env python3
"""
XLerobot ACT Model Inference Script

This script runs ACT policy inference on XLerobot hardware.
Based on the SO100 inference pipeline with proper normalization.
"""

import json
import logging
import sys
import time
from pathlib import Path

import torch
import yaml

from sst_xlerbot.teleop.quest_vr_xlerobot_controller_no_base import XLerobotArmOnly

try:
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
    from lerobot.cameras.configs import ColorMode, Cv2Rotation
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
    from lerobot.datasets.pipeline_features import aggregate_pipeline_dataset_features, create_initial_features
    from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts
    from lerobot.policies.factory import make_policy, make_pre_post_processors
    from lerobot.policies.utils import make_robot_action
    from lerobot.processor import make_default_processors
    from lerobot.robots.xlerobot import XLerobot
    from lerobot.robots.xlerobot.config_xlerobot import XLerobotConfig
    from lerobot.utils.constants import ACTION, OBS_STR
    from lerobot.utils.control_utils import predict_action
except ModuleNotFoundError:
    PROJECT_ROOT = Path(__file__).resolve().parents[4]
    LEROBOT_SRC = PROJECT_ROOT / "lerobot" / "src"
    sys.path.insert(0, str(LEROBOT_SRC))
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
    from lerobot.cameras.configs import ColorMode, Cv2Rotation
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
    from lerobot.datasets.pipeline_features import aggregate_pipeline_dataset_features, create_initial_features
    from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts
    from lerobot.policies.factory import make_policy, make_pre_post_processors
    from lerobot.policies.utils import make_robot_action
    from lerobot.processor import make_default_processors
    from lerobot.robots.xlerobot import XLerobot
    from lerobot.robots.xlerobot.config_xlerobot import XLerobotConfig
    from lerobot.utils.constants import ACTION, OBS_STR
    from lerobot.utils.control_utils import predict_action

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    force=True
)
logger = logging.getLogger(__name__)

# Suppress noisy logs
logging.getLogger('lerobot.robots.utils').setLevel(logging.ERROR)


def precise_sleep(duration_s: float):
    """Sleep for precise duration"""
    if duration_s > 0:
        time.sleep(duration_s)


def load_config(config_path: str) -> dict:
    """Load configuration from YAML file"""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def setup_robot(config: dict) -> XLerobot:
    """Initialize and connect to XLerobot"""
    logger.info("Initializing XLerobot...")

    # Build camera configs
    camera_configs = {}
    for cam_name, cam_cfg in config['cameras'].items():
        if not cam_cfg.get('enabled', True):
            continue
        camera_configs[cam_name] = OpenCVCameraConfig(
            index_or_path=cam_cfg['index'],
            fps=cam_cfg['fps'],
            width=cam_cfg['width'],
            height=cam_cfg['height'],
            color_mode=ColorMode.RGB,
            rotation=Cv2Rotation.NO_ROTATION,
        )

    # Create robot config
    robot_cfg = XLerobotConfig(
        port1=config['robot']['port1'],
        port2=config['robot']['port2'],
        use_degrees=config['robot'].get('use_degrees', True),
        cameras=camera_configs,
    )

    use_head = config['robot'].get('use_head', True)
    robot = XLerobotArmOnly(robot_cfg, use_head=use_head)

    logger.info("Connecting to robot...")
    robot.connect()

    logger.info("✅ Robot connected successfully")
    return robot


def run_inference_episode(
    robot,
    policy,
    preprocessor,
    postprocessor,
    dataset_features,
    robot_observation_processor,
    robot_action_processor,
    device,
    max_steps: int,
    fps: int,
    task_name: str,
    debug_log_interval: int = 10,
    debug_logger=None,
):
    """
    Run a single episode of inference.

    This follows the same pipeline as lerobot-record but without dataset collection.
    """
    logger.info(f"\n{'='*60}")
    logger.info(f"Starting new episode (max {max_steps} steps)")
    logger.info(f"{'='*60}\n")

    # Reset policy and processors
    policy.reset()
    preprocessor.reset()
    postprocessor.reset()

    step = 0
    start_episode_t = time.perf_counter()
    period = 1.0 / fps

    while step < max_steps:
        start_loop_t = time.perf_counter()

        try:
            # Get robot observation
            obs = robot.get_observation()

            # Apply observation processor pipeline
            obs_processed = robot_observation_processor(obs)

            # Build observation frame
            observation_frame = build_dataset_frame(dataset_features, obs_processed, prefix=OBS_STR)

            # Predict action using policy
            action_values = predict_action(
                observation=observation_frame,
                policy=policy,
                device=device,
                preprocessor=preprocessor,
                postprocessor=postprocessor,
                use_amp=policy.config.use_amp if hasattr(policy.config, 'use_amp') else False,
                task=task_name,
                robot_type=robot.robot_type,
            )

            # Convert to robot action format
            act_processed_policy = make_robot_action(action_values, dataset_features)

            # Apply action processor pipeline
            robot_action_to_send = robot_action_processor((act_processed_policy, obs))

            # Send action to robot
            _sent_action = robot.send_action(robot_action_to_send)

            # Debug logging
            if step % debug_log_interval == 0:
                logger.info(f"[Step {step}] Running inference...")

                # Log arm positions
                left_motors = ['left_arm_shoulder_pan', 'left_arm_shoulder_lift', 'left_arm_elbow_flex']
                right_motors = ['right_arm_shoulder_pan', 'right_arm_shoulder_lift', 'right_arm_elbow_flex']

                left_info = ", ".join([
                    f"{m.split('_')[-1]}={obs.get(f'{m}.pos', 0):.1f}"
                    for m in left_motors
                ])
                right_info = ", ".join([
                    f"{m.split('_')[-1]}={obs.get(f'{m}.pos', 0):.1f}"
                    for m in right_motors
                ])

                logger.info(f"  Left arm:  {left_info}")
                logger.info(f"  Right arm: {right_info}")

            step += 1

            if debug_logger is not None:
                debug_logger.log_step(
                    episode_start=start_episode_t,
                    step=step,
                    obs=obs,
                    action_policy=act_processed_policy,
                    action_sent=robot_action_to_send,
                )

            # Maintain control loop frequency
            dt_s = time.perf_counter() - start_loop_t
            precise_sleep(period - dt_s)

        except KeyboardInterrupt:
            logger.info("\n⏹️  Interrupted by user")
            raise
        except Exception as e:
            logger.error(f"Error in step {step}: {e}", exc_info=True)
            time.sleep(0.1)
            continue

    episode_duration = time.perf_counter() - start_episode_t
    logger.info(f"\n✅ Episode finished: {step} steps in {episode_duration:.1f}s")


def main():
    # Get script directory for relative paths
    script_dir = Path(__file__).parent

    # Load inference configuration
    config_path = script_dir / "config_inference.yaml"
    logger.info(f"Loading config from: {config_path}")
    config = load_config(config_path)

    # Extract configs
    inference_cfg = config['inference']
    model_path = script_dir / inference_cfg['model_path']
    dataset_id = inference_cfg['dataset_id']
    device = torch.device(inference_cfg['device'])
    max_episodes = inference_cfg['max_episodes']
    max_steps = inference_cfg['max_steps_per_episode']
    fps = inference_cfg['fps']
    task_name = inference_cfg.get('task_name', 'Pick and place')

    logger.info(f"{'='*60}")
    logger.info(f"XLerobot ACT Inference")
    logger.info(f"{'='*60}")
    logger.info(f"Model: {model_path}")
    logger.info(f"Dataset: {dataset_id}")
    logger.info(f"Device: {device}")
    logger.info(f"{'='*60}\n")

    robot = None
    debug_logger = None

    try:
        # Load dataset metadata for normalization stats
        logger.info(f"Loading dataset metadata from {dataset_id}...")
        dataset_metadata = LeRobotDatasetMetadata(dataset_id)
        logger.info("✅ Dataset metadata loaded")

        # Load policy configuration and model
        logger.info(f"\nLoading policy from {model_path}...")
        policy_cfg = PreTrainedConfig.from_pretrained(model_path)
        policy_cfg.device = str(device)
        policy_cfg.pretrained_path = str(model_path)
        policy = make_policy(policy_cfg, ds_meta=dataset_metadata)
        policy.eval()
        logger.info("✅ Policy loaded successfully")

        # Create preprocessor and postprocessor with normalization
        logger.info("\nSetting up pre/post processors...")
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg,
            pretrained_path=policy_cfg.pretrained_path,
            dataset_stats=dataset_metadata.stats,
            preprocessor_overrides={"device_processor": {"device": policy_cfg.device}},
        )
        logger.info("✅ Processors ready")

        # Setup robot
        logger.info("\nSetting up robot...")
        robot = setup_robot(config)

        # Create processor pipelines (same as lerobot-record)
        logger.info("\nCreating processor pipelines...")
        teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

        # Create dataset features (same as lerobot-record)
        logger.info("Creating dataset features...")
        dataset_features = combine_feature_dicts(
            aggregate_pipeline_dataset_features(
                pipeline=teleop_action_processor,
                initial_features=create_initial_features(action=robot.action_features),
                use_videos=False,
            ),
            aggregate_pipeline_dataset_features(
                pipeline=robot_observation_processor,
                initial_features=create_initial_features(observation=robot.observation_features),
                use_videos=True,
            ),
        )
        logger.info(f"✅ Dataset features created: {len(dataset_features)} features")

        # Optional debug logger
        debug_log_file = inference_cfg.get('debug_log_file')
        if debug_log_file:
            debug_logger = StepLogger(script_dir / debug_log_file)

        # Run inference episodes
        logger.info("\n" + "="*60)
        logger.info(f"Starting inference loop: {max_episodes} episodes")
        logger.info("="*60)

        for episode_idx in range(max_episodes):
            logger.info(f"\n>>> Episode {episode_idx + 1}/{max_episodes}")

            run_inference_episode(
                robot=robot,
                policy=policy,
                preprocessor=preprocessor,
                postprocessor=postprocessor,
                dataset_features=dataset_features,
                robot_observation_processor=robot_observation_processor,
                robot_action_processor=robot_action_processor,
                device=device,
                max_steps=max_steps,
                fps=fps,
                task_name=task_name,
                debug_log_interval=inference_cfg.get('debug_log_interval', 10),
                debug_logger=debug_logger,
            )

            time.sleep(0.5)  # Brief pause between episodes

        logger.info("\n🎉 Inference completed!")

    except KeyboardInterrupt:
        logger.info("\n⏹️  Script interrupted by user")
    except Exception as e:
        logger.error(f"❌ Error occurred: {e}", exc_info=True)
        sys.exit(1)
    finally:
        if robot is not None:
            try:
                logger.info("\nDisconnecting robot...")
                robot.disconnect()
                logger.info("✅ Robot disconnected")
            except Exception as e:
                logger.error(f"Error during disconnect: {e}")

        if debug_logger is not None:
            debug_logger.close()

        logger.info("\n✅ Script finished")


class StepLogger:
    """Logger for debugging inference vs recording comparison"""

    def __init__(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = str(path)
        self.file = open(path, "w", buffering=1)
        logger.info(f"📝 Debug logging to: {path}")

    def log_step(self, episode_start: float, step: int, obs: dict, action_policy: dict, action_sent: dict):
        def extract_state(d: dict) -> dict:
            return {k: float(v) for k, v in d.items() if k.endswith(".pos") and isinstance(v, (int, float))}

        record = {
            "timestamp": time.perf_counter() - episode_start,
            "step": step,
            "observation": extract_state(obs),
            "action_policy": extract_state(action_policy),
            "action_sent": extract_state(action_sent),
        }
        self.file.write(json.dumps(record) + "\n")

    def __del__(self):
        self.close()

    def close(self):
        try:
            if hasattr(self, "file") and self.file:
                self.file.close()
                self.file = None
        except Exception:
            pass


if __name__ == "__main__":
    main()
