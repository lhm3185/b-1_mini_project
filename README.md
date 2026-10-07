# b-1_mini_project

ROKEY 부트캠프 미니 프로젝트. 고정 웹캠이 RC카를 발견하면 AMR(TurtleBot4)이 지정 좌표로 가서 RC카를 찾고, 일정 거리를 유지하며 따라간다.

| 패키지 | 역할 |
|---|---|
| `vision_pkg` | 고정 웹캠 RC카 검출, AMR 카메라(OAK-D)로 RC카 위치 산출 |
| `navi_pkg` | 검출 신호 수신, 지정 좌표 이동, scan_motion, RC카 추종 |
| `interface_pkg` | 두 패키지가 주고받는 메시지·서비스 정의 (`msg/`, `srv/`) |

패키지 사이의 토픽 약속은 **[docs/interfaces.md](docs/interfaces.md)** 에 있다. 구현 전에 먼저 읽는다.
터미널별 실행 순서는 **[docs/run_guide.md](docs/run_guide.md)** 에 있다.

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

## 실행

PC 한 대에서 터미널 두 개로 띄운다(고정 웹캠도 그 PC 에 꽂는다). 자세한 순서는 [docs/run_guide.md](docs/run_guide.md).

```bash
# 터미널 1 — AMR: 위치 추정, Nav2, RViz, 주행 노드. 로봇은 도킹 상태에서 시작
ros2 launch navi_pkg navi.launch.py

# 터미널 2 — 비전: AMR 캠, 고정 웹캠 (검출 화면 창 두 개)
ros2 launch vision_pkg vision.launch.py

# 추종을 끝내고 dock 시키기 (종료 알림음을 낸다)
ros2 service call /robot4/stop_follow std_srvs/srv/Trigger
```

웹캠이 RC카를 보면 AMR 이 알림음을 내고 undock → 지정 좌표 이동 → 회전하며 탐색 → 찾으면 유지 거리를 지키며 추종 → 중단 요청(또는 `follow_sec` 경과) 뒤 dock 앞으로 돌아와 dock 하고 다음 신호를 기다린다.
웹캠 노드는 응답을 받으면 스스로 끝난다. 다시 시연하려면 터미널 2 를 다시 띄운다.

| 런치 | 인자 | 기본값 | 뜻 |
|---|---|---|---|
| `navi.launch.py` | `map` | `~/maps/my_map.yaml` | 지도 파일 |
| | `nav2` | true | false 면 위치 추정·Nav2 를 띄우지 않는다(이미 떠 있을 때) |
| | `rviz` | true | RViz |
| | `follow_sec` | 600.0 | 출발부터 이 시간이 지나면 복귀(초) |
| `vision.launch.py` | `webcam` | true | false 면 AMR 캠만 (웹캠이 다른 PC 에 있을 때) |
| | `compressed_depth`, `best_effort` | true | AMR 캠이 압축 depth 를 BEST_EFFORT 로 받는다(Wi-Fi 가 붐빌 때 끊김 방지) |
| | `camera_index` | 2 | 고정 웹캠의 `/dev/video` 번호 |
| 공통 | `namespace` | `/robot4` | 로봇 네임스페이스 |

AMR 캠 모델은 `vision_pkg/models/amrcam_yolo26n.pt` 에 둔다(웹캠 모델 `webcam_yolo26n.pt` 와 같은 폴더).

| 노드 | 파라미터 | 기본값 | 뜻 |
|---|---|---|---|
| `amr_cam` | `model_path` | `share/vision_pkg/models/amrcam_yolo26n.pt` | YOLO 모델 파일 |
| | `conf` | 0.85 | `car` 검출 신뢰도 기준 |
| | `show_window` | false (런치에서는 true) | 박스와 거리를 그린 영상을 창으로 띄운다 |
| | `car_height` | 0.0 | RC카 높이(m). 넣으면 박스 높이로 구한 거리를 로그에 같이 찍는다(비교용) |
| | `best_effort` | false | 영상 구독 QoS 를 BEST_EFFORT 로 |
| `follow_car` | `keep_distance` | 0.8 | 유지 거리(m, 로봇 중심 기준). 0.64 m 보다 가까우면 depth 가 나오지 않는다 |
| | `follow_sec` | 600.0 | 출발부터 이 시간이 지나면 복귀 |
| | `go_to_goal` | true | false 면 undock 한 자리에서 바로 탐색 |
| | `goal_x`, `goal_y` | -3.88, -2.84 | 탐색을 시작할 지도 좌표 |
| | `dock_front_x`, `dock_front_y` | -1.0, -0.07 | 복귀할 dock 앞 좌표 |

확인:

```bash
ros2 topic echo /robot4/amr_state            # IDLE / MOVING / SCANNING / FOLLOWING
ros2 topic echo /robot4/rc_car_target
ros2 service call /robot4/rc_car_detected interface_pkg/srv/WebcamDetection "{detected: true}"   # 웹캠 없이 출발시키기
```
