# SST XLeRobot - Quest VR Teleoperation

이 패키지는 **[XLeRobot](https://github.com/Vector-Wangel/XLeRobot)** 을 기반으로, Meta Quest VR로 XLeRobot을 제어하는 텔레오퍼레이션 및 데이터셋 녹화 패키지입니다.

---

## ⚠️ 시작 전 필수: 하드웨어 조립

**이 repo는 하드웨어 조립이 완료된 상태를 전제로 합니다.**

조립이 아직 안 되어 있다면 아래 영상을 먼저 따라 완성해 주세요:

▶️ **[XLeRobot 조립 가이드 영상](https://www.youtube.com/watch?v=upB1CEFeOlk)**

조립 완료 후 이 README를 진행하세요.

---

## 📁 디렉토리 구조

```
sst_xlerobot/
├── src/sst_xlerbot/
│   ├── teleop/
│   │   ├── quest_vr_xlerobot_controller_no_base.py  # 양팔 + 머리 (바퀴 없음)
│   │   └── quest_vr_xlerobot_controller.py          # 양팔 + 머리 + 바퀴 (풀버전)
│   ├── recording/
│   │   ├── quest_vr_record_dataset_v2.py            # 데이터셋 녹화 CLI
│   │   └── config_recording.yaml                    # 녹화 설정 파일
│   ├── quest/
│   │   └── quest_socket_monitor.py                  # Quest 소켓 수신
│   ├── model/
│   │   └── SO101Robot.py                            # IK/FK 키네마틱스
│   └── inference/
│       ├── run_xlerobot_inference.py                # 정책 추론
│       └── config_inference.yaml                    # 추론 설정 파일
├── tests/                                           # 테스트 스크립트
├── lerobot/                                         # lerobot fork (Git submodule)
├── pyproject.toml
├── install.sh
├── run.sh
└── README.md
```

---

## 🔁 클론 & 서브모듈 초기화

```bash
# 최초 클론 시 (서브모듈 포함)
git clone --recurse-submodules https://github.com/XHAKA3456/sst_xlerobot.git
cd sst_xlerobot

# 이미 클론했다면
git submodule update --init --recursive
```

---

## 🚀 빠른 시작 (자동 설치)

```bash
# 자동 설치 (conda 환경 생성 + 의존성 설치)
./install.sh

# 실행 (바퀴 없음: 양팔 + 머리)
./run.sh

# 실행 (풀버전: 양팔 + 머리 + 바퀴)
./run.sh --base
```

---

## 🔧 수동 설치

### 1. Conda 환경 설정

```bash
conda create -n xlerobot python=3.10 -y
conda activate xlerobot
```

### 2. 의존성 설치 (순서 중요!)

```bash
# ⭐ 1단계: lerobot[feetech] 먼저 설치
cd lerobot
pip install -e ".[feetech]"
cd ..

# ⭐ 2단계: sst_xlerbot 설치
pip install -e .
```

### 3. 시스템 권한 설정 (Linux)

```bash
sudo usermod -a -G dialout $USER
# 로그아웃 후 다시 로그인 필요
```

### 4. 하드웨어 연결 확인

```bash
ls /dev/ttyACM*    # /dev/ttyACM0  /dev/ttyACM1 출력되어야 함
ls /dev/video*     # 카메라 확인
```

---

## ▶️ 텔레오퍼레이션

### 실행

```bash
# 양팔 + 머리만 (바퀴 없음)
./run.sh

# 양팔 + 머리 + 바퀴 (풀버전)
./run.sh --base
```

또는 직접 실행:

```bash
conda activate xlerobot

# no_base (팔 + 머리)
python -m sst_xlerbot.teleop.quest_vr_xlerobot_controller_no_base

# 풀버전 (팔 + 머리 + 바퀴)
python -m sst_xlerbot.teleop.quest_vr_xlerobot_controller
```

### 동작 방식

| Quest 입력 | 로봇 동작 |
|-----------|---------|
| 왼쪽 컨트롤러 위치/회전 | 왼팔 IK 제어 |
| 오른쪽 컨트롤러 위치/회전 | 오른팔 IK 제어 |
| 컨트롤러 트리거 | 그리퍼 열기/닫기 |
| 헤드셋 회전 (yaw/pitch) | 로봇 머리(pan/tilt) 제어 |
| 조이스틱 (풀버전만) | 3축 옴니휠 이동 |

### 설정 파일 위치

```
src/sst_xlerbot/teleop/quest_vr_xlerobot_controller_no_base.py  # 상단 상수 수정
src/sst_xlerbot/teleop/quest_vr_xlerobot_controller.py          # 상단 상수 수정
```

수정 가능한 상수:

```python
CAMERA_INDEX = 2       # Quest로 스트리밍할 카메라 장치 인덱스
VIDEO_PORT = 5656      # Quest로 비디오 스트리밍 포트
# Quest 데이터 수신 포트: 5454
```

---

## 🎥 데이터셋 녹화

LeRobot 공식 포맷으로 데이터셋을 수집합니다.

### 실행

```bash
conda activate xlerobot
sst-record --config src/sst_xlerbot/recording/config_recording.yaml
```

### 설정 파일

`src/sst_xlerbot/recording/config_recording.yaml`

```yaml
dataset:
  repo_id: "your_hf_username/your_dataset_name"  # Hugging Face 업로드 ID
  task: "Pick and place the doll"                 # 태스크 설명
  output_dir: "./dataset"                         # 로컬 저장 경로
  push_to_hub: true                               # HF Hub 업로드 여부

recording:
  num_episodes: 20      # 녹화할 에피소드 수
  episode_time: 30      # 에피소드당 최대 시간 (초)
  fps: 30               # 녹화 프레임레이트
  reset_time: 15        # 에피소드 간 리셋 시간 (초)

robot:
  port1: "/dev/ttyACM0"   # 왼팔 + 머리 포트
  port2: "/dev/ttyACM1"   # 오른팔 포트
  use_head: false          # 머리 모터 포함 여부
  use_base: false          # 바퀴 포함 여부 (true면 풀버전 컨트롤러 사용)

cameras:
  head_camera:
    index: 2              # 카메라 장치 인덱스
    quest_only: true      # true면 Quest 전송만 (데이터셋 저장 제외)
  left:
    index: 4
  right:
    index: 6
```

> 카메라 인덱스는 `ls /dev/video*` 로 확인 후 수정하세요.

### 녹화 중 조작

- **에피소드 자동 시작**: 설정된 `episode_time` 동안 자동 녹화
- **왼팔 특수 동작**: Quest `lift` 버튼 > 0.5 → 왼팔 특수 포즈 실행
- **오른팔 특수 동작**: Quest `agv.x` > 0.5 → 오른팔 특수 포즈 실행
- **종료**: `Ctrl+C`

---

## 🤖 추론 (Inference)

학습된 ACT 정책으로 로봇을 자율 제어합니다.

### 실행

```bash
conda activate xlerobot
python -m sst_xlerbot.inference.run_xlerobot_inference
```

### 설정 파일

`src/sst_xlerbot/inference/config_inference.yaml`

```yaml
robot:
  port1: "/dev/ttyACM0"
  port2: "/dev/ttyACM1"
  use_head: false
  use_base: false          # 바퀴 포함 여부

cameras:
  head_camera:
    index: 2
    enabled: true
  left:
    index: 4
    enabled: true
  right:
    index: 6
    enabled: false         # 훈련 시 사용한 카메라만 활성화

inference:
  model_path: "./models/xlerobot_act_pickplace"   # 학습된 모델 경로
  dataset_id: "your_hf_username/your_dataset"     # 정규화 통계 로드용
  task_name: "Pick and place the red cube"
  device: "cuda"           # "cuda", "cpu", "mps"
  max_episodes: 1000
  max_steps_per_episode: 500
  fps: 30
```

### 모델 준비

```
src/sst_xlerbot/inference/models/
└── xlerobot_act_pickplace/
    ├── config.json
    ├── model.safetensors
    └── ...
```

Hugging Face Hub에서 다운로드하거나 `lerobot-train`으로 학습한 모델을 위 경로에 배치하세요.

---

## 🔧 설정 요약

> **⚠️ 카메라 인덱스, USB 포트, 시리얼 포트는 PC 환경마다 다릅니다.**
> `ls /dev/video*` 와 `ls /dev/ttyACM*` 으로 실제 할당된 장치를 먼저 확인한 후, 아래 파일들을 본인 환경에 맞게 수정하세요.

| 파일 | 수정 항목 |
|------|---------|
| `recording/config_recording.yaml` | 카메라 인덱스, 포트, `use_head`, `use_base`, HF repo ID |
| `inference/config_inference.yaml` | 카메라 인덱스, 포트, `use_head`, `use_base`, 모델 경로 |
| `teleop/quest_vr_xlerobot_controller*.py` | `CAMERA_INDEX`, `VIDEO_PORT` (상단 상수) |

### Quest VR 앱 설정

- PC와 Quest를 같은 네트워크에 연결
- Quest 앱에서 PC의 IP 주소 입력
- 포트 **5454**: VR 컨트롤러 데이터 전송
- 포트 **5656**: 카메라 영상 수신

---

## 📋 하드웨어 요구사항

| 항목 | 사양 |
|------|------|
| 로봇 | XLeRobot (Feetech STS3215 모터) |
| USB 변환기 | 2개 (`/dev/ttyACM0`, `/dev/ttyACM1`) |
| VR | Meta Quest 헤드셋 + 컨트롤러 |
| 카메라 | USB 카메라 (권장 1280×720) |

**버스 구성**
- `port1 (ttyACM0)`: 왼팔 (ID 1-6) + 헤드 (ID 7-8)
- `port2 (ttyACM1)`: 오른팔 (ID 1-6) + 바퀴 (ID 7-9, 풀버전만)

---

## 🐛 문제 해결

### ImportError: No module named 'lerobot'
```bash
cd lerobot && pip install -e ".[feetech]"
```

### Serial Port Permission Denied
```bash
sudo usermod -a -G dialout $USER  # 재로그인 필요
# 또는 임시
sudo chmod 666 /dev/ttyACM0 /dev/ttyACM1
```

### Camera Not Found
```bash
ls /dev/video*          # 인덱스 확인 후
# config_recording.yaml 또는 config_inference.yaml의 cameras.*.index 수정
```

### Quest Connection Timeout
- PC와 Quest가 같은 네트워크인지 확인
- 방화벽에서 포트 5454, 5656 허용
- Quest 앱에서 PC IP 주소 정확히 입력

### Dataset Directory Already Exists
```bash
# 기존 데이터셋 삭제 후 재시작
rm -rf ./dataset/your_hf_username_your_dataset_name
# 또는 config_recording.yaml에서 resume: true 설정
```

---

## 🔍 주요 기능

- ✅ Quest 양손 컨트롤러 → 양팔 IK 제어
- ✅ Quest 헤드셋 회전 → 로봇 머리(pan/tilt) 제어
- ✅ Quest 조이스틱 → 바퀴 이동 (풀버전)
- ✅ USB 카메라 → Quest 실시간 스트리밍
- ✅ 컨트롤러 + 헤드셋 캘리브레이션
- ✅ 안전한 관절 각도 제한 및 부드러운 초기화
- ✅ LeRobot 공식 포맷 데이터셋 수집 (HF Hub 업로드 지원)
- ✅ ACT 정책 추론 (학습된 모델 실행)
- ✅ `use_base` / `use_head` 옵션으로 구성 유연하게 선택

---

## 📝 크레딧

- **[XLeRobot](https://github.com/Vector-Wangel/XLeRobot)** — 로봇 하드웨어 및 기반 설계
- **[LeRobot](https://github.com/huggingface/lerobot)** — 로봇 제어 라이브러리 (Apache-2.0)
