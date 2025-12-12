# SST XLeRobot - Quest VR Teleoperation

이 패키지는 Quest VR을 사용하여 XLeRobot을 제어하는 완전한 standalone 버전입니다.

## 📁 디렉토리 구조

```
sst_xlerbot/
├── pyproject.toml                          # 패키지 메타데이터 (setuptools)
├── src/
│   └── sst_xlerbot/
│       ├── recording/
│       │   └── quest_vr_record_dataset_v2.py   # 레코딩 CLI (메인)
│       ├── teleop/
│       │   └── quest_vr_xlerobot_controller_no_base.py
│       ├── quest/
│       │   └── quest_socket_monitor.py
│       ├── model/
│       │   └── SO101Robot.py
│       └── xlerobot_src/
│           └── ...                                   # XLeRobot 파생 소스
├── recording/config_recording.yaml
├── dataset/                                          # 녹화 결과가 저장되는 기본 위치
├── lerobot/                                         # lerobot fork (Git submodule)
└── README.md
```

## 🔁 클론 & 서브모듈 초기화

```bash
# 최초 클론 시
git clone --recurse-submodules https://github.com/XHAKA3456/sst_xlerobot.git
cd sst_xlerobot

# 이미 클론했다면
git submodule update --init --recursive
```

## 🚀 빠른 시작 (자동 설치)

가장 쉬운 방법:

```bash
# 자동 설치 스크립트 실행
./install.sh

# 프로그램 실행
./run.sh
```

## 🔧 수동 설치

### 1. Conda 환경 설정

```bash
# Miniconda가 설치되어 있어야 합니다
conda --version

# xlerobot 환경 생성
conda create -n xlerobot python=3.10 -y
conda activate xlerobot
```

### 2. 의존성 설치 (중요: 순서대로!)

```bash
# ⭐ 1단계: lerobot[feetech] 먼저 설치
# (이게 torch, datasets, feetech-servo-sdk 등 모든 기본 의존성을 제공합니다)
cd lerobot
pip install -e ".[feetech]"
cd ..

# ⭐ 2단계: sst_xlerbot 설치
# (rerun-sdk 등 추가 의존성만 설치되며, lerobot과 충돌 없음)
pip install -e .
```

**의존성 구조:**
- `lerobot[feetech]` 제공: numpy, opencv-python, torch, datasets, feetech-servo-sdk, etc.
- `sst_xlerbot` 추가: rerun-sdk (visualization용)
- **중복 없음!** 동일한 패키지를 공유하여 충돌 방지

### 3. 시스템 권한 설정 (Linux만 해당)

```bash
# USB 시리얼 포트 접근 권한
sudo usermod -a -G dialout $USER
# 로그아웃 후 다시 로그인 필요
```

### 4. 하드웨어 연결 확인

```bash
# USB 시리얼 포트 확인
ls /dev/ttyACM*
# 출력: /dev/ttyACM0  /dev/ttyACM1

# 카메라 확인
ls /dev/video*
```

### 5. 실행

```bash
# 방법 1: run 스크립트 사용
./run.sh

# 방법 2: 직접 실행
conda activate xlerobot
python src/sst_xlerbot/teleop/quest_vr_xlerobot_controller_no_base.py
```

## 🔧 설정

### 포트 설정 변경

`src/sst_xlerbot/teleop/quest_vr_xlerobot_controller_no_base.py` 파일에서:

```python
# 시리얼 포트 (main 함수 내)
robot_config = XLerobotConfig(use_degrees=True)
# 기본값: port1="/dev/ttyACM0", port2="/dev/ttyACM1"

# 카메라 인덱스
CAMERA_INDEX = 2  # 파일 상단에서 변경

# 네트워크 포트
VIDEO_PORT = 5656      # Quest로 비디오 스트리밍
quest_monitor = QuestSocketMonitor(host="0.0.0.0", port=5454)  # Quest VR 데이터 수신
```

### Quest VR 앱 설정

Quest VR 앱에서 다음 설정이 필요합니다:
- PC IP 주소로 연결
- 포트 5454로 VR 데이터 전송
- 포트 5656에서 비디오 수신

## 📋 하드웨어 요구사항

### 필수
- **로봇**: XLerobot (Feetech STS3215 모터)
- **컨트롤러**:
  - USB to Serial 변환기 2개 (`/dev/ttyACM0`, `/dev/ttyACM1`)
  - 왼팔 + 헤드: port1
  - 오른팔: port2 (베이스 모터는 이 버전에서 제외)

### 선택사항
- **카메라**: USB 카메라 (1280x720 해상도 지원)
- **VR**: Meta Quest 헤드셋 + 컨트롤러

## 🐛 문제 해결

### ImportError: No module named 'lerobot'
```bash
cd lerobot
pip install -e ".[feetech]"
```

### Serial Port Permission Denied
```bash
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
# 또는
sudo usermod -a -G dialout $USER  # 재로그인 필요
```

### Camera Not Found
```bash
# 카메라 인덱스 확인
ls /dev/video*
# 메인 스크립트에서 CAMERA_INDEX 값 수정
```

### Quest Connection Timeout
- PC와 Quest가 같은 네트워크에 있는지 확인
- 방화벽에서 포트 5454, 5656 허용
- Quest 앱에서 올바른 IP 주소 설정

## 📦 다른 PC로 이동 (예: 라즈베리파이)

전체 `sst_xlerbot` 폴더를 압축하여 이동:

```bash
# 현재 PC에서 압축 (lerobot 포함)
cd /path/to/parent_directory
tar -czf sst_xlerbot.tar.gz sst_xlerbot/

# 라즈베리파이로 전송
scp sst_xlerbot.tar.gz pi@raspberrypi.local:~/

# 라즈베리파이에서
cd ~
tar -xzf sst_xlerbot.tar.gz
cd sst_xlerbot

# 자동 설치
./install.sh

# 또는 수동 설치 (위의 "수동 설치" 섹션 참조)
```

**주의사항:**
- ✅ lerobot과 sst_xlerbot을 **반드시 순서대로** 설치하세요
- ✅ 의존성 충돌 없음 (동일한 torch, datasets 등을 공유)
- ✅ config 파일로 하드웨어 경로 관리 (`recording/config_recording.yaml`)

## 🔍 주요 기능

- ✅ Quest VR 양손 컨트롤러로 양팔 제어
- ✅ 헤드셋 회전으로 로봇 머리 제어
- ✅ 실시간 IK (Inverse Kinematics) 계산
- ✅ USB 카메라 → Quest로 비디오 스트리밍
- ✅ 안전한 관절 각도 제한
- ✅ 부드러운 초기화 (3초)
- ✅ 컨트롤러 캘리브레이션

## 📝 라이센스

이 프로젝트는 LeRobot (Apache-2.0 License)과 XLeRobot을 기반으로 합니다.

## 🙋 지원

문제가 발생하면 메인 스크립트의 디버그 메시지를 확인하세요.
50 프레임마다 상태 정보가 출력됩니다.
