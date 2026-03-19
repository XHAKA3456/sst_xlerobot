# SST XLeRobot - Quest VR Teleoperation

This package is built on **[XLeRobot](https://github.com/Vector-Wangel/XLeRobot)** and provides Meta Quest VR teleoperation, dataset recording, and policy inference for XLeRobot.

---

## ⚠️ Before You Start: Hardware Assembly

**This repo assumes hardware assembly is already complete.**

If you haven't assembled the robot yet, follow this video first:

▶️ **[XLeRobot Assembly Guide](https://www.youtube.com/watch?v=upB1CEFeOlk)**

Come back to this README after assembly is done.

---

## 📁 Directory Structure

```
sst_xlerobot/
├── src/sst_xlerbot/
│   ├── teleop/
│   │   ├── quest_vr_xlerobot_controller_no_base.py  # Arms + head (no wheels)
│   │   └── quest_vr_xlerobot_controller.py          # Arms + head + wheels (full)
│   ├── recording/
│   │   ├── quest_vr_record_dataset_v2.py            # Dataset recording CLI
│   │   └── config_recording.yaml                    # Recording config
│   ├── quest/
│   │   └── quest_socket_monitor.py                  # Quest socket receiver
│   ├── model/
│   │   └── SO101Robot.py                            # IK/FK kinematics
│   └── inference/
│       ├── run_xlerobot_inference.py                # Policy inference
│       └── config_inference.yaml                    # Inference config
├── tests/                                           # Test scripts
├── lerobot/                                         # lerobot fork (Git submodule)
├── pyproject.toml
├── install.sh
├── run.sh
└── README.md
```

---

## 🔁 Clone & Initialize Submodules

```bash
# First-time clone (includes submodules)
git clone --recurse-submodules https://github.com/XHAKA3456/sst_xlerobot.git
cd sst_xlerobot

# If already cloned without submodules
git submodule update --init --recursive
```

---

## 🚀 Quick Start (Auto Install)

```bash
# Auto install (creates conda env + installs dependencies)
./install.sh

# Run teleop (no wheels: arms + head)
./run.sh

# Run teleop (full: arms + head + wheels)
./run.sh --base
```

---

## 🔧 Manual Installation

### 1. Create Conda Environment

```bash
conda create -n xlerobot python=3.10 -y
conda activate xlerobot
```

### 2. Install Dependencies (order matters!)

```bash
# ⭐ Step 1: Install lerobot[feetech] first
cd lerobot
pip install -e ".[feetech]"
cd ..

# ⭐ Step 2: Install sst_xlerbot
pip install -e .
```

### 3. Serial Port Permissions (Linux)

```bash
sudo usermod -a -G dialout $USER
# Log out and log back in
```

### 4. Verify Hardware Connection

```bash
ls /dev/ttyACM*    # Should show /dev/ttyACM0  /dev/ttyACM1
ls /dev/video*     # Verify cameras
```

---

## ▶️ Teleoperation

### Run

```bash
# Arms + head only (no wheels)
./run.sh

# Full version (arms + head + wheels)
./run.sh --base
```

Or run directly:

```bash
conda activate xlerobot

# No base (arms + head)
python -m sst_xlerbot.teleop.quest_vr_xlerobot_controller_no_base

# Full version (arms + head + wheels)
python -m sst_xlerbot.teleop.quest_vr_xlerobot_controller
```

### Control Mapping

| Quest Input | Robot Action |
|-------------|-------------|
| Left controller position/rotation | Left arm IK control |
| Right controller position/rotation | Right arm IK control |
| Controller trigger | Gripper open/close |
| Headset rotation (yaw/pitch) | Head pan/tilt control |
| Joystick (full version only) | 3-axis omniwheel movement |

### Configuration

Edit the constants at the top of the teleop scripts:

```
src/sst_xlerbot/teleop/quest_vr_xlerobot_controller_no_base.py
src/sst_xlerbot/teleop/quest_vr_xlerobot_controller.py
```

```python
CAMERA_INDEX = 2       # Camera device index for Quest streaming
VIDEO_PORT = 5656      # Video streaming port to Quest
# Quest data receive port: 5454
```

---

## 🎥 Dataset Recording

Records datasets in the official LeRobot format.

### Run

```bash
conda activate xlerobot
sst-record --config src/sst_xlerbot/recording/config_recording.yaml
```

### Config File

`src/sst_xlerbot/recording/config_recording.yaml`

```yaml
dataset:
  repo_id: "your_hf_username/your_dataset_name"  # Hugging Face upload ID
  task: "Pick and place the doll"                 # Task description
  output_dir: "./dataset"                         # Local save path
  push_to_hub: true                               # Upload to HF Hub

recording:
  num_episodes: 20      # Number of episodes to record
  episode_time: 30      # Max time per episode (seconds)
  fps: 30               # Recording framerate
  reset_time: 15        # Reset time between episodes (seconds)

robot:
  port1: "/dev/ttyACM0"   # Left arm + head port
  port2: "/dev/ttyACM1"   # Right arm port
  use_head: false          # Include head motors
  use_base: false          # Include wheels (true = full robot controller)

cameras:
  head_camera:
    index: 2              # Camera device index
    quest_only: true      # true = stream to Quest only, exclude from dataset
  left:
    index: 4
  right:
    index: 6
```

> Check camera indices with `ls /dev/video*` and update accordingly.

### Controls During Recording

- **Episode auto-starts**: records automatically for `episode_time` seconds
- **Left arm special pose**: Quest `lift` button > 0.5 → triggers left arm special pose
- **Right arm special pose**: Quest `agv.x` > 0.5 → triggers right arm special pose
- **Stop**: `Ctrl+C`

---

## 🤖 Inference

Run a trained ACT policy to autonomously control the robot.

### Run

```bash
conda activate xlerobot
python -m sst_xlerbot.inference.run_xlerobot_inference
```

### Config File

`src/sst_xlerbot/inference/config_inference.yaml`

```yaml
robot:
  port1: "/dev/ttyACM0"
  port2: "/dev/ttyACM1"
  use_head: false
  use_base: false          # Include wheels

cameras:
  head_camera:
    index: 2
    enabled: true
  left:
    index: 4
    enabled: true
  right:
    index: 6
    enabled: false         # Enable only cameras used during training

inference:
  model_path: "./models/xlerobot_act_pickplace"   # Path to trained model
  dataset_id: "your_hf_username/your_dataset"     # For loading normalization stats
  task_name: "Pick and place the red cube"
  device: "cuda"           # "cuda", "cpu", or "mps"
  max_episodes: 1000
  max_steps_per_episode: 500
  fps: 30
```

### Model Setup

Place your trained model at:

```
src/sst_xlerbot/inference/models/
└── xlerobot_act_pickplace/
    ├── config.json
    ├── model.safetensors
    └── ...
```

Download from Hugging Face Hub or use a model trained with `lerobot-train`.

---

## 🔧 Configuration Summary

| File | What to Edit |
|------|-------------|
| `recording/config_recording.yaml` | Camera indices, ports, `use_head`, `use_base`, HF repo ID |
| `inference/config_inference.yaml` | Camera indices, ports, `use_head`, `use_base`, model path |
| `teleop/quest_vr_xlerobot_controller*.py` | `CAMERA_INDEX`, `VIDEO_PORT` (top-level constants) |

### Quest VR App Settings

- Connect PC and Quest to the same network
- Enter the PC's IP address in the Quest app
- Port **5454**: VR controller data (Quest → PC)
- Port **5656**: Camera video stream (PC → Quest)

---

## 📋 Hardware Requirements

| Item | Spec |
|------|------|
| Robot | XLeRobot (Feetech STS3215 motors) |
| USB adapters | 2x (`/dev/ttyACM0`, `/dev/ttyACM1`) |
| VR | Meta Quest headset + controllers |
| Camera | USB camera (recommended 1280×720) |

**Bus Layout**
- `port1 (ttyACM0)`: Left arm (ID 1-6) + Head (ID 7-8)
- `port2 (ttyACM1)`: Right arm (ID 1-6) + Wheels (ID 7-9, full version only)

---

## 🐛 Troubleshooting

### ImportError: No module named 'lerobot'
```bash
cd lerobot && pip install -e ".[feetech]"
```

### Serial Port Permission Denied
```bash
sudo usermod -a -G dialout $USER  # Then re-login
# Or temporarily:
sudo chmod 666 /dev/ttyACM0 /dev/ttyACM1
```

### Camera Not Found
```bash
ls /dev/video*   # Check available indices, then update:
# cameras.*.index in config_recording.yaml or config_inference.yaml
```

### Quest Connection Timeout
- Verify PC and Quest are on the same network
- Allow ports 5454 and 5656 through your firewall
- Double-check the PC IP address entered in the Quest app

### Dataset Directory Already Exists
```bash
# Delete existing dataset and restart
rm -rf ./dataset/your_hf_username_your_dataset_name
# Or set resume: true in config_recording.yaml
```

---

## 🔍 Features

- ✅ Quest dual controllers → dual arm IK control
- ✅ Quest headset rotation → robot head (pan/tilt) control
- ✅ Quest joystick → omniwheel movement (full version)
- ✅ USB camera → real-time streaming to Quest
- ✅ Controller + headset calibration
- ✅ Safe joint angle limits and smooth initialization
- ✅ LeRobot-format dataset recording (HF Hub upload support)
- ✅ ACT policy inference (run trained models)
- ✅ Flexible `use_base` / `use_head` config options

---

## 📝 Credits

- **[XLeRobot](https://github.com/Vector-Wangel/XLeRobot)** — Robot hardware and base design
- **[LeRobot](https://github.com/huggingface/lerobot)** — Robot learning library (Apache-2.0)
