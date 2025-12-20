# Quest VR Teleoperation for XLeRobot

Quest VR 헤드셋을 사용한 XLeRobot 양팔 로봇 원격 조작 시스템입니다.

## 개요

이 디렉토리는 **패키지화되지 않은 standalone 스크립트**로, Quest VR 헤드셋과 컨트롤러를 사용하여 XLeRobot 하드웨어를 직접 제어합니다.

### 주요 기능

- **양방향 통신**:
  - **Quest → Robot**: VR 컨트롤러 위치/회전 데이터를 로봇 조인트 명령으로 변환
  - **Robot → Quest**: USB 카메라 영상을 실시간으로 Quest 헤드셋에 스트리밍

- **제어 방식**:
  - 왼쪽/오른쪽 컨트롤러 → 왼팔/오른팔 제어 (IK 기반)
  - 조이스틱 → 베이스 이동 (전진/후진/회전)
  - 헤드셋 회전 → 머리 팬/틸트
  - 트리거 → 그리퍼 개폐

## 파일 구조

```
vr_teleop_xlerobot/
├── quest_vr_xlerobot_hw_refactored_safe_2link.py  # 메인 스크립트
├── quest_socket_monitor.py                         # Quest VR 데이터 수신 서버
├── model/
│   └── SO101Robot.py                               # 2-link IK 계산
└── README.md
```

## 필요한 환경

### 하드웨어
- XLeRobot (양팔 로봇)
- Quest VR 헤드셋 (Quest 2/3/Pro)
- USB 카메라 (영상 스트리밍용)

### 소프트웨어 의존성
- Python 3.10+
- lerobot 패키지 (from `/home/stream/sst_xlerbot/lerobot/src`)
- OpenCV (`cv2`)
- NumPy
- PySerial (로봇 통신)

## 실행 방법

### 1. 로봇 하드웨어 연결 확인

로봇이 연결되어 있는지 확인:
```bash
ls /dev/ttyACM*
# 예상 출력: /dev/ttyACM0, /dev/ttyACM1, ...
```

카메라 연결 확인:
```bash
ls /dev/video*
# 예상 출력: /dev/video0, /dev/video2, ...
```

### 2. Quest VR 앱 실행

Quest 헤드셋에서 VR teleoperation 앱을 실행하고, PC의 IP 주소로 연결합니다.

**연결 포트**:
- Quest 데이터 수신: `5454` (TCP)
- 영상 스트리밍: `5656` (TCP)

### 3. 스크립트 실행

```bash
cd /home/stream/sst_xlerbot/vr_teleop_xlerobot
python quest_vr_xlerobot_hw_refactored_safe_2link.py
```

### 4. 초기화 프로세스

스크립트 실행 후 자동으로 진행되는 단계:

1. **Quest 연결 대기** (최대 60초)
2. **로봇 초기 자세 이동** (3초)
3. **VR 장치 캘리브레이션**:
   - 왼쪽 컨트롤러 (2초)
   - 오른쪽 컨트롤러 (2초)
   - 헤드셋 (2초)
4. **텔레오퍼레이션 시작** (50Hz)

## 통신 프로토콜

### Quest → Robot

**데이터 형식** (JSON via TCP socket):
```json
{
  "left": {
    "enabled": true,
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "euler": {"x": 0.0, "y": 0.0, "z": 0.0},
    "trigger": 0.5
  },
  "right": { /* 동일 구조 */ },
  "head": {
    "euler": {"x": 0.0, "y": 0.0, "z": 0.0}
  },
  "agv": {"x": 0.0, "y": 0.0}
}
```

### Robot → Quest

**영상 스트리밍** (MJPEG over TCP):
1. 4바이트 크기 정보 (big-endian uint32)
2. JPEG 프레임 데이터

**설정**:
- 해상도: 1280x720
- JPEG 품질: 85
- 프레임 드롭: 큐 full 시 자동

## 제어 파라미터

스크립트 내 `QuestVRXLeRobotController` 클래스에서 조정 가능:

```python
self.position_scale = 0.5         # 위치 이동 스케일
self.rotation_scale = 1.0         # 회전 스케일
self.base_linear_scale = 0.2      # 베이스 전진/후진 속도
self.base_angular_scale = 40.0    # 베이스 회전 속도
self.head_pan_scale = 1.0         # 머리 팬 스케일
self.head_tilt_scale = 1.0        # 머리 틸트 스케일
```

## 문제 해결

### Quest가 연결되지 않음
- PC와 Quest가 같은 네트워크에 있는지 확인
- 방화벽이 5454, 5656 포트를 차단하지 않는지 확인
- PC의 IP 주소를 Quest 앱에 정확히 입력했는지 확인

### 로봇이 움직이지 않음
- `/dev/ttyACM*` 포트가 존재하는지 확인
- 사용자가 dialout 그룹에 속해 있는지 확인:
  ```bash
  sudo usermod -aG dialout $USER
  # 로그아웃 후 다시 로그인
  ```

### 영상이 전송되지 않음
- 카메라 인덱스 확인 (기본값: `CAMERA_INDEX = 2`)
- 다른 프로그램이 카메라를 사용 중인지 확인

## 참고 사항

- 이 스크립트는 **하드웨어 직접 제어**를 위해 MuJoCo 시뮬레이션 의존성을 제거한 버전입니다
- IK는 2-link 평면 IK를 사용하며, `SO101Robot.py`에 구현되어 있습니다
- 초기 자세는 zero position (모든 조인트 0도, 그리퍼 33.3% 열림)입니다
- 제어 루프는 50Hz로 실행됩니다

## 라이선스

이 프로젝트는 XLeRobot 프로젝트의 일부입니다.
