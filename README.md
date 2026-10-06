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

## 실행

### AMR 캠 노드 (`vision_pkg`)

GPU PC 에서 `rokey_venv` 를 켠 상태로 실행한다. **처음부터 띄워 둔다** — 구독을 시작해야 로봇 카메라가 depth 를 만들기 시작하고, 첫 영상까지 3~5 초가 걸린다. 로봇이 도킹 중이면 카메라가 꺼져 있어 "영상 없음" 이 찍히고, undock 하면 자동으로 이어진다.

```bash
ros2 run vision_pkg amr_cam --ros-args -r __ns:=/robot4 \
  -r /tf:=/robot4/tf -r /tf_static:=/robot4/tf_static \
  -p model_path:=$HOME/Downloads/amrcam_yolo26n.pt
```

`/tf` 리매핑 두 개를 빼면 로봇 좌표를 찾지 못해 "TF 대기 중" 만 찍힌다.

| 파라미터 | 기본값 | 뜻 |
|---|---|---|
| `model_path` | (필수) | YOLO 모델 파일 |
| `conf` | 0.85 | `car` 검출 신뢰도 기준 |
| `hold_sec` | 1.0 | 마지막 검출 뒤 이 시간이 지나면 `detected = false` |
| `car_height` | 0.0 | RC카 높이(m). 넣으면 박스 높이로 구한 거리를 로그에 같이 찍는다(비교용) |
| `best_effort` | false | 영상 구독 QoS 를 BEST_EFFORT 로 (Wi-Fi 가 붐빌 때 비교용) |

확인:

```bash
ros2 service call /robot4/rc_car_target interface_pkg/srv/RcCarTarget
ros2 run rqt_image_view rqt_image_view /robot4/rc_car_debug/compressed   # 박스와 거리가 그려진 영상
```

