# b-1_mini_project

ROKEY 부트캠프 미니 프로젝트. 고정 웹캠이 RC카를 발견하면 AMR(TurtleBot4)이 지정 좌표로 가서 RC카를 찾고, 일정 거리를 유지하며 따라간다.

| 패키지 | 역할 |
|---|---|
| `vision_pkg` | 고정 웹캠 RC카 검출, AMR 카메라(OAK-D)로 RC카 위치 산출 |
| `navi_pkg` | 검출 신호 수신, 지정 좌표 이동, scan_motion, RC카 추종 |
| `interface_pkg` | 두 패키지가 주고받는 메시지·서비스 정의 (`msg/`, `srv/`) |

패키지 사이의 토픽 약속은 **[docs/interfaces.md](docs/interfaces.md)** 에 있다. 구현 전에 먼저 읽는다.

## 받기 · 빌드

강의 환경(`rokey_venv`, `~/rokey_ws`) 기준이다.

```bash
cd ~/rokey_ws/src
git clone https://github.com/lhm3185/b-1_mini_project.git
cd ~/rokey_ws
colcon build --symlink-install --packages-select interface_pkg navi_pkg vision_pkg
source install/setup.bash
```

브랜치를 따로 만들지 않고 `main` 에 바로 올린다. 올리기 전에 `git pull` 을 먼저 한다.

## 실행 (브랜치 `feature/rc_car_follow`)

세 노드를 띄운다. 순서는 상관없지만 **AMR 캠 노드는 처음부터 띄워 둔다**(구독을 시작해야 로봇 카메라가 depth 를 만들고, 첫 영상까지 3~5 초가 걸린다). 로봇이 도킹 중이면 카메라가 꺼져 있어 "영상 없음" 이 찍히고, undock 하면 자동으로 이어진다.

```bash
# 1) AMR 캠 (GPU PC, rokey_venv)  — /tf 리매핑 두 개를 빼면 "TF 대기 중" 만 찍힌다
ros2 run vision_pkg amr_cam --ros-args -r __ns:=/robot4 \
  -r /tf:=/robot4/tf -r /tf_static:=/robot4/tf_static \
  -p model_path:=<amrcam_yolo26n.pt 경로>

# 2) AMR 주행 (Nav2·localization 이 켜져 있어야 한다. 로봇은 도킹 상태에서 시작)
ros2 run navi_pkg rc_car_follower --ros-args -r __ns:=/robot4

# 3) 고정 웹캠 (웹캠이 꽂힌 PC)
ros2 run vision_pkg webcam_node
```

웹캠이 RC카를 보면 AMR 이 알림음을 내고 undock → 지정 좌표 이동 → 회전하며 탐색 → 찾으면 유지 거리를 지키며 추종 → `follow_sec` 뒤 dock 앞으로 돌아와 dock 한다.

| 노드 | 파라미터 | 기본값 | 뜻 |
|---|---|---|---|
| `amr_cam` | `model_path` | `share/vision_pkg/models/amrcam_yolo26n.pt` | YOLO 모델 파일 |
| | `conf` | 0.85 | `car` 검출 신뢰도 기준 |
| | `car_height` | 0.0 | RC카 높이(m). 넣으면 박스 높이로 구한 거리를 로그에 같이 찍는다(비교용) |
| | `best_effort` | false | 영상 구독 QoS 를 BEST_EFFORT 로 |
| `rc_car_follower` | `keep_distance` | 0.8 | 유지 거리(m, 로봇 중심 기준). 0.64 m 보다 가까우면 depth 가 나오지 않는다 |
| | `follow_sec` | 60.0 | 출발부터 이 시간이 지나면 복귀 |
| | `go_to_goal` | true | false 면 undock 한 자리에서 바로 탐색 |
| | `goal_x`, `goal_y` | -3.88, -2.84 | 탐색을 시작할 지도 좌표 |
| | `dock_front_x`, `dock_front_y` | -1.0, -0.07 | 복귀할 dock 앞 좌표 |

확인:

```bash
ros2 topic echo /robot4/rc_car_target
ros2 run rqt_image_view rqt_image_view /robot4/rc_car_debug/compressed   # 박스와 거리가 그려진 영상
ros2 service call /robot4/rc_car_detected interface_pkg/srv/WebcamDetection "{detected: true}"   # 웹캠 없이 출발시키기
ros2 service call /robot4/stop_follow std_srvs/srv/Trigger   # 추종을 끝내고 dock 으로 복귀시키기
```

