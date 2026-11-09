# Y_version_2 — AutoRace 2026, 워크스페이스 하나 · 패키지 하나 · 명령 한 줄

이 repo가 그대로 ROS 2 워크스페이스다 (`src/autorace` 패키지 하나). 원본 repo들은 건드리지 않고 아래 커밋에서 복사했다.

| 원본 (읽기만 함) | 커밋 | 여기로 온 것 |
| --- | --- | --- |
| `lane_detection_dl` (선배) | `f1dcce7` | YOLO 모델 2개 (`lane_yolo26n_seg.pt`, `yolo11n_mission.pt`), 표지판 인식 노드 `mission_model` → `autorace/sign` |
| `camera_ros` (선배) | `cf15991` | `/camera` 퍼블리셔(보정), `/camera2` 앞 카메라 퍼블리셔, 뷰어, `camera_calibration.pkl` → `autorace/camera` |
| `lane_detection_ad` | `fd83702` | 차선 노드 → `autorace/lane` |
| `missions_v1` | `7f8ce90` | 미션 FSM + cmd_vel mux → `autorace/mission` |

처음 합칠 때 바꾼 건 import 경로, 모델·설정 파일 위치(`share/autorace/...`)뿐이었다. 그 뒤 이 repo에서 바꾼 것:
- 미션 중에 다른 표지판이 떠서 대기열이 만료돼도, 그 표지판이 아직 보이고 있으면 앞 미션이 끝난 뒤 시작한다.
- 카메라를 번호 대신 고정 장치 이름으로 연다 (아래 "카메라 두 대 지정").
- 교차로 출구를 헤딩으로 판정한다 (아래 "교차로 미션이 하는 일").
- 터널: LiDAR로 지도를 쌓으며 출구까지 경로 계획, 기본으로 켜짐 (아래 "터널 미션이 하는 일").

## 설치 (처음 한 번)

```bash
git clone https://github.com/turtlebot-autorace-2026/Y_version_2.git ~/Y_version_2
cd ~/Y_version_2
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
source ~/Y_version_2/install/setup.bash        # ~/.bashrc 에 넣어 두면 편함
```

- `ultralytics`(YOLO)가 필요하다. 선배처럼 `~/autorace/yolo_venv`에 깔려 있으면 YOLO 노드 2개를 자동으로 그 파이썬으로 돌린다. 없으면 시스템 `python3`를 쓴다. 다른 위치는 `python:=/경로/python`.
- 패키지 이름은 `autorace` 하나뿐이다. 다른 워크스페이스(`camera_ros`, `lane_detection_ad` 등)를 같이 source해도 이름이 안 겹친다.

## 실차 — 로봇에서 한 줄

```bash
ros2 launch autorace robot.launch.py
```

로봇(OpenCR `/dev/ttyACM0`가 있는 컴퓨터)에서 실행하면 아래가 전부 같이 뜬다.

| 노드 | 하는 일 |
| --- | --- |
| turtlebot3_bringup | `/odom`, `/scan`, 바퀴 (`/cmd_vel`) |
| camera_publisher / camera_publisher2 | `/camera` 차선 카메라(보정), `/camera2` 앞 카메라 — 장치는 `config/camera_params.yaml` |
| sign_detection (YOLO11n) | `/parking` `/left` `/right` `/stop` `/construction` `/intersection` `/tunnel` |
| lane_detection (YOLO26n-seg) | 차선 주행 명령 → `/cmd_vel_lane` |
| mission_fsm | 신호등 → 표지판에 따라 교차로 · 공사 · 주차 · 차단바 · 터널 |
| cmd_vel_mux | 선택된 쪽 명령만 `/cmd_vel`로 (끊기면 정지) |

시작하면 빨간불을 본 뒤 초록불에 출발한다.

### 카메라 두 대 지정 (로봇마다 처음 한 번)

선배 코드는 카메라를 번호(`0` = 차선, `2` = 앞)로 열었다. 이 번호는 USB가 잡히는 순서라 재부팅·재연결 후 바뀔 수 있고, 그러면 차선 노드가 앞 카메라를 본다. 토픽 연결(`/camera` = 차선, `/camera2` = 표지판·신호등)은 맞으니, 장치를 바뀌지 않는 이름으로 고정한다.

```bash
# robot.launch 를 끈 상태에서 (켜져 있으면 카메라가 사용 중이라 못 연다)
ros2 run autorace list_cameras
```

장치마다 `/dev/v4l/by-path/...`, `/dev/v4l/by-id/...` 이름과 해상도가 나오고, 스냅샷이 `~/autorace_cam_check/videoN.jpg`에 저장된다. 바로 앞 바닥이 보이는 쪽이 차선 카메라다. 출력 끝에 나온 `device:` 줄을 `src/autorace/config/camera_params.yaml`에 넣고 `colcon build`를 다시 한다. 두 카메라가 같은 모델이면 by-path(USB 포트 기준)를 쓰고, 카메라를 꽂는 포트를 바꾸지 않는다.

한 번만 바꿔 보려면 `lane_camera:=/dev/v4l/by-path/... front_camera:=/dev/v4l/by-path/...`를 쓴다. 실행 로그 첫 줄에서 `lane camera: ... -> /dev/videoN`으로 실제로 연 장치를 확인한다.

| 인자 | 기본값 | 뜻 |
| --- | --- | --- |
| `start_mode` | `light` | `go` = 신호등 안 보고 바로 출발 |
| `lane_camera` / `front_camera` | `''` | 이번 실행만 카메라 장치 지정 (기본: `config/camera_params.yaml`) |
| `intersection_on_turn_sign` | `false` | `true` = 삼각 교차로 표지판 없이 좌/우 표지판만으로 교차로 미션 시작 (연습맵 전용, 대회 코스에서는 주차 쪽 좌회전 화살표에도 걸린다) |
| `v_max` | `0.18` | 직선 속도 m/s (커브는 알아서 감속) |
| `bringup` / `cameras` | `auto` | auto = 로봇일 때만 켬 (`true`/`false`로 강제) |
| `python` | 자동 | YOLO 노드용 파이썬 |
| `device` | `''` | YOLO 장치 (`cpu`, `0`) |
| `show` | `false` | cv2 창 (화면 필요) |

YOLO를 노트북에서 돌리고 싶으면: 로봇에서 `ros2 launch turtlebot3_bringup robot.launch.py`, `ros2 run autorace camera_publisher`, `ros2 run autorace camera_publisher2`를 띄운다. 노트북에서는 `ros2 launch autorace robot.launch.py`를 실행한다 (노트북에는 OpenCR이 없어서 bringup과 카메라는 알아서 건너뜀).

## Gazebo — 한 줄

```bash
ros2 launch autorace sim.launch.py
```

hakee의 `autorace_gazebo`가 설치(source)돼 있으면 Gazebo까지 같이 뜬다. Gazebo를 따로 켰으면 `gazebo:=false`.
- 카메라 변환: `/camera/image_raw` → `/camera`, `/front_camera/image_raw` → `/camera2`
- 스폰 위치: `x:= y:= yaw:=`, 창 없이 돌리려면 `gui:=false`, 속도는 `v_max` (기본 0.18)
- Gazebo 로봇에 앞 카메라가 없으면 표지판도 신호등도 안 보인다. 이때는 5초 뒤 신호등 없이 출발하고, 미션은 시작되지 않는다(차선 주행만 함).

## 교차로 미션이 하는 일

1. 삼각 교차로 표지판(`/intersection`)이 뜨면 감속(`v_max_intersection`)하고 좌/우 표지판을 기다린다. 좌/우가 **둘 다** 보이면 정하지 않고 계속 기다린다 (대회에서는 하나만 나온다).
2. 하나가 뜨면 그 분기의 **바깥선만** 따라간다 (왼쪽 = 노란선, 오른쪽 = 흰선, `mission_params.yaml`의 `left_line` / `right_line`).
3. 출구 판정은 표지판이 사라진 거리가 아니라 오도메트리 헤딩으로 한다. 분기 쪽으로 꺾고(약 90°), 섬 끝을 반대로 돌고(약 180°), 다시 분기 쪽으로 꺾어 출구에 들어가면(약 90°) 0.25 m 더 바깥선을 따라간 뒤 양쪽 선으로 돌아간다.
4. 5 m 안에 이 패턴이 안 나오면 그냥 양쪽 선으로 돌아간다 (`max_m`).

예전 규칙(표지판이 사라지고 1 m 뒤 양쪽 선)은 로봇이 아직 링 안, 섬 반대편에 있을 때 양쪽 선으로 돌려놨다. 출구 입구에서 차선 노드는 짝이 안 맞는 선 쌍 중 노란선을 남기는데, 섬 쪽 선이 노란색인 분기에서는 그게 섬이라 한 바퀴 더 돌았다.

`ros2 topic echo /mission/state`에 `intersection/branch left u=.. peak=..`처럼 단계(`armed` → `branch` → `far_side` → `exit`)와 회전 각도가 나온다.

## 터널 미션이 하는 일

안은 어둡고 장애물은 무작위라 카메라 없이 LiDAR와 오도메트리만 쓴다. 2025년 팀 코드(`tunnel.zip`)는 미리 만든 지도와 Nav2로 출구 좌표(시작점에서 앞 1.42 m, 왼쪽 1.86 m, +93°)까지 갔다. 그 지도와 Nav2 설정은 없어서, 지도를 미리 만들지 않고 터널 안에서 쌓는다.

1. `/tunnel` 표지판 → 차선 주행. 차선이 끊기면 헤딩을 유지하고 직진하다가, 양옆 0.35 m 안에 벽이 잡히는 곳(입구)에서 터널 좌표계를 잡는다.
2. 스캔마다 LiDAR 점을 3 cm 격자에 쌓는다 (모르는 칸은 빈 칸으로 봄). 단 0.8 m 안의 점만 쌓고, 직전 스캔 이후 4° 넘게 돌았으면 그 스캔은 쌓지 않는다. LDS-01은 한 바퀴 0.2 s(5 Hz)라 도는 중에 찍힌 스캔은 번져 있고, 0.5 m 넘으면 거리 오차가 ±3.5 %인데 격자는 한 번 쌓인 점을 지우지 않기 때문이다.
3. 1초마다(그리고 새로 본 장애물이 경로를 막으면 바로) 격자 위에서 출구 바깥 목표까지 경로를 다시 계획한다. 장애물에서 0.11 m 안으로는 못 가고 0.25 m까지는 비용을 더 매겨 떨어져 간다.
4. 경로 0.25 m 앞 점을 향해 가되, 지금 스캔에서 그 방향이 막혀 있으면 가장 가까운 열린 방향으로 간다.
5. 출구 개구부 안에 들어와서 차선이 다시 보이면 차선 주행으로 넘긴다. 10초 동안 경로상 진척이 없으면 오른쪽 벽따라가기 0.6 m 후 다시 계획한다. 120초가 넘으면 그냥 차선으로 넘긴다.

**대회 터널에서 꼭 잴 것** (`mission_params.yaml`, 지금 값은 Gazebo 맵):
- `exit_fwd` = 입구 벽 **바깥면**에서 출구 차선 중심까지 진행 방향으로 잰 거리 **+ 약 0.1 m**. 좌표 원점이 양옆 벽이 0.35 m 안에 처음 잡히는 곳, 즉 바깥면보다 약 0.1 m 앞(입구 폭에 따라 0.09~0.11)이라서다. Gazebo는 1.90 + 0.11 = 2.01.
- `exit_left` = 들어오는 차선 중심선에서 출구가 뚫린 벽 안쪽 면까지 옆으로 잰 거리. 출구가 오른쪽이면 음수. Gazebo는 1.95.

0.2 m 정도 틀려도 격자가 개구부를 직접 찾는다(테스트 있음). 크게 틀리면 경로가 엉뚱한 벽으로 향한다.

`/mission/state`에 `tunnel/goal ... plans=N`과 지금 고른 방향, 남은 여유 거리가 나온다.

## 출발 램프 (cmd_vel_mux)

정지 뒤(모드 stop, 소스 침묵, FSM 사망) 다시 움직일 때 전진 속도는 `accel`(기본 0.3 m/s², 0.18 m/s까지 0.6 s)보다 빨리 오르지 못한다. 각속도는 같은 비율로 줄여서 소스가 요청한 회전 반경을 유지한다. 차선 노드는 되돌려받을 때마다 추정기를 재시작해 첫 프레임부터 최고 속도를 내기 때문이다. 감속·후진은 제한하지 않는다. 2025년 팀 driving 노드의 프레임당 0.03 m/s 제한을 시간 기준으로 옮긴 것이다. `accel: 0.0`이면 끈다.

## 2025년 팀 코드(driving_ninja)에서 아직 안 가져온 것 — Gazebo 로그 보고 결정

선배 repo는 수정하지 않는다. 아래 둘은 로그에서 해당 증상이 보일 때만 옮긴다.

| 항목 | 선배 방식 | 우리 현재 | 로그에서 볼 것 |
| --- | --- | --- | --- |
| 회전 표지판 ROI | `/left`·`/right`가 뜨면 BEV ROI를 중앙선부터 그쪽 영상 끝까지, 높이 1.4배, 30 s 유지 | `follow_line`으로 한쪽 선만 따라감, ROI 폭 ±250 px 그대로 | 교차로 `branch`~`exit` 구간의 `/lane_detection/status`에 `lost`나 `memory`가 연속으로 나오는지. 나오면 바깥쪽 선이 ROI를 벗어난 것 |
| 실차 ROI/목표점 | `roi_top_row` 150 (기본 220), `lookahead_row` 300 (기본 400)로 낮춰 더 멀리 봄. 실차 튜닝 흔적(`#220` 주석) | 220 / 제어기 다름(근·원 분리) | 실차에서 `/lane_detection/debug/compressed`의 ROI 상자 안에 선이 직선에서 10줄 이상 잡히는지. 모자라면 `roi_top_row`를 내린다 |

## 보는 법

```bash
ros2 topic echo /mission/state                                   # 지금 미션 / 표지판 / 출력 한 줄
ros2 run rqt_image_view rqt_image_view /mission/debug/compressed # 신호등·차단바 판정 화면
ros2 run rqt_image_view rqt_image_view /mission_model/debug/compressed   # 표지판 박스
ros2 run rqt_image_view rqt_image_view /lane_detection/debug/compressed  # 차선
```

## 검증한 것 / 안 한 것

- pytest 74개 통과 (`cd src/autorace && python3 -m pytest -q test`)
  - 차선 추정 8개
  - 미션 FSM과 mux (교차로: 섬을 돌아 출구까지 헤딩 흐름, 접근로 굽음·흔들림, max_m)
  - 카메라 장치 이름 해석 (가짜 /dev 트리)
  - 맵 기하로 돌린 2D 시뮬: 공사, 주차, 터널 (Gazebo 터널 + 규정 1.8 m 터널, 장애물 주머니 배치 4개, 안쪽 길이 없을 때 터널 밖으로 안 나감, 출구 좌표 ±0.2 m 오차)
  - 같은 2D 시뮬에 실차 효과를 넣은 것 (`simlib.Robot`): 오도메트리 배율 오차, LiDAR 5 Hz 스윕 번짐, LDS-01 스펙 수준 거리 노이즈
    - 터널: 7개 배치 × 12개 조건에서 75/84 → 81/84. 남은 3개는 가장 좁은 통로가 210 mm(로봇 200 mm)인 배치
    - 주차: 오도메트리 yaw ±3 %에서 이미 차체 모서리가 주차선에 걸렸다(38/60). 스퍼에서 `/lane/centre` 헤딩으로 기준 방향을 다시 잡고, 바퀴축이 아니라 차체를 칸 중앙에 맞춘 뒤 60/60, 선까지 최소 +8 mm(yaw·거리 −6 %), 그 외 +24 mm 이상
    - 공사: 모든 조건 통과 (변경 없음)
  - **실차에서 잴 것**: 오도메트리 배율. 1 m 직진·제자리 2바퀴를 `/odom`과 줄자로 비교. 바퀴 실측 Ø70 mm인데 펌웨어가 Ø66 mm(반지름 0.033)를 쓰면 거리·회전이 약 6 % 작게 찍힌다. 주차 여유가 이 조건에서 가장 작다 (+8 mm)
  - 색 규칙
- launch 파일 2개는 ROS 2 Galactic의 실제 `launch` / `launch_ros` 코드로 펼쳐서 각 노드의 실행 명령과 파라미터를 확인했다.
  - venv 파이썬 prefix
  - `/cmd_vel_lane`
  - `use_sim_time`
  - Gazebo 스폰 인자 전달
  - bringup·카메라 자동 판단
- colcon 방식으로 설치한 트리에서 노드 전부를 가짜 ROS/카메라/YOLO로 띄워 봤다.
  - 보정 파일, 모델 2개 경로 확인
  - 표지판 → FSM 연결 확인
- **실제 ROS · Gazebo · 로봇에서는 아직 안 돌렸다.** 기본값(신호등 ROI, 공사·주차 거리 등)은 Gazebo 맵에서 잰 값이라 현장 조정이 필요할 수 있다. 각 파라미터 설명은 `src/autorace/config/mission_params.yaml`.

## 구성

```text
src/autorace/
  autorace/camera/    선배 camera_ros (publisher, publisher2, viewer)
  autorace/sign/      선배 mission_model (표지판 YOLO11n)
  autorace/lane/      lane_detection_ad (차선 YOLO26n-seg + 제어, Gazebo 카메라 변환)
  autorace/mission/   missions_v1 (FSM, 미션, mux, LiDAR/비전 도구)
  config/             lane_params.yaml, mission_params.yaml, camera_params.yaml, camera_calibration.pkl
  models/             lane_yolo26n_seg.pt, yolo11n_mission.pt
  launch/             robot.launch.py, sim.launch.py
  test/               pytest
```
