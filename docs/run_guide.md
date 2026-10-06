# 실행 가이드 (터미널별)

RC카가 고정 웹캠에 보이면 AMR 이 출발해 RC카를 찾아 따라가고, 중단을 요청하면 dock 으로 돌아오는 시나리오를 터미널에서 직접 돌리는 순서다.
2026-10-06 에 실제 로봇(`/robot4`)으로 확인한 구성 그대로 적었다. 토픽·서비스의 뜻은 [interfaces.md](interfaces.md) 를 본다.

## 0. 한눈에 보기

| PC | 터미널 | 띄우는 것 | 준비됐다는 표시 |
|---|---|---|---|
| GPU PC 1 (MSI, `mu-04`) | 1 | 위치 추정 (localization) | 로그가 멈추고 오류가 없다 |
| | 2 | Nav2 | 〃 |
| | 3 | AMR 캠 노드 `amr_cam` | `영상 없음 (도킹 중이면 카메라가 꺼져 있다)` 가 5 초마다 |
| | 4 | AMR 주행 노드 `rc_car_follower` | `웹캠 검출 신호 대기 (서비스 rc_car_detected)` |
| | 5 | 중단 명령, 상태 확인용 | — |
| GPU PC 2 (VICTUS, `hv-04`) | 1 | 고정 웹캠 노드 `webcam_node` | 웹캠 창이 뜨고 `SEARCHING 0/3` |

순서는 **MSI 1 → 2 → 3 → 4, 그다음 VICTUS 1** 이다. 웹캠 노드는 RC카가 보이는 순간 바로 출발 신호를 보내므로 맨 마지막에 띄운다.

## 1. 시작 전 확인

- 로봇이 **도킹 스테이션에 올라가 있다.** 주행 노드는 도킹 상태에서 시작한다.
- 두 PC 가 Wi-Fi `turtle08` 에 붙어 있다.
- 외장 웹캠이 VICTUS 에 꽂혀 있다. `ls /dev/video*` 에 `/dev/video2` 가 보여야 한다(0, 1 은 내장 카메라).
- RC카는 **웹캠 시야 밖**에 둔다. 더미는 어디 있어도 된다.
- AMR 팀이 Nav2 를 이미 띄워 두었으면 MSI 터미널 1, 2 는 건너뛴다. 확인:
  ```bash
  ros2 lifecycle get /robot4/bt_navigator     # active [3] 이면 떠 있는 것
  ```

## 2. 코드 받기와 빌드 (PC 마다 처음 한 번)

두 GPU PC 에는 10/6 에 `~/amr_cam_test_ws` 로 받아 빌드해 둔 것이 있다. 그대로 쓰려면 최신으로만 맞춘다.

```bash
cd ~/amr_cam_test_ws/src/b-1_mini_project
git pull                      # 브랜치 feature/rc_car_follow
cd ~/amr_cam_test_ws
colcon build --symlink-install
```

새로 받을 때:

```bash
mkdir -p ~/amr_cam_test_ws/src && cd ~/amr_cam_test_ws/src
git clone -b feature/rc_car_follow https://github.com/lhm3185/b-1_mini_project.git
cd ~/amr_cam_test_ws
colcon build --symlink-install
```

`which colcon` 이 `~/venvs/rokey_venv/...` 여야 한다. 시스템 colcon 으로 빌드하면 노드가 venv 의 ultralytics 를 찾지 못한다.

## 3. 터미널마다 먼저 칠 것

새 터미널을 열 때마다 한 번 친다. (`.bashrc` 가 앞의 네 줄을 이미 하고 있으면 마지막 줄만 치면 된다.)

```bash
source ~/venvs/rokey_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/turtlebot4_ws/install/setup.bash
source /etc/turtlebot4_discovery/setup.bash
source ~/amr_cam_test_ws/install/setup.bash     # 반드시 마지막에
```

마지막 줄이 맨 뒤에 와야 한다. 다른 워크스페이스에 옛 판 `interface_pkg` 가 빌드돼 있으면 그쪽이 먼저 잡혀 메시지 타입이 맞지 않는다.

## 4. MSI (`mu-04`)

### 터미널 1 — 위치 추정

```bash
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot4 map:=/home/mu-04/maps/my_map.yaml
```

### 터미널 2 — Nav2

```bash
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot4
```

지도와 로봇 위치를 보고 싶으면 터미널을 하나 더 열어 RViz 를 띄운다(선택).

```bash
ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=/robot4
```

### 터미널 3 — AMR 캠 노드

```bash
ros2 run vision_pkg amr_cam --ros-args -r __ns:=/robot4 \
  -r /tf:=/robot4/tf -r /tf_static:=/robot4/tf_static \
  -p model_path:=$HOME/Downloads/rc_car.v4i.yolov8/runs/car_train_20261002_190815/weights/best.pt
```

- `amr_cam 시작: conf=0.85, ...` 다음에 `영상 없음 (도킹 중이면 카메라가 꺼져 있다)` 가 5 초마다 찍히면 정상이다. 로봇이 undock 하면 `영상 수신 중` 으로 바뀐다. 노드를 다시 띄울 필요가 없다.
- `/tf` 리매핑 두 개를 빼면 `TF 대기 중` 만 계속 찍히고 값이 나오지 않는다.
- VICTUS 에서 띄울 때 모델 경로는 `$HOME/Downloads/amrcam_yolo26n.pt` 다(같은 모델).

### 터미널 4 — AMR 주행 노드

```bash
ros2 run navi_pkg rc_car_follower --ros-args -r __ns:=/robot4 -p follow_sec:=900.0
```

- `웹캠 검출 신호 대기 (서비스 rc_car_detected)` 가 찍히면 준비된 것이다. 로봇은 아직 움직이지 않는다.
- `follow_sec` 은 출발부터 이 시간이 지나면 스스로 돌아오게 하는 상한이다(초). 빼면 60 초다.

| 파라미터 | 기본값 | 뜻 |
|---|---|---|
| `keep_distance` | 0.8 | RC카와 유지할 거리(m, 로봇 중심 기준). 0.64 m 보다 작게 주면 거리 값이 나오지 않는다 |
| `follow_sec` | 60.0 | 출발부터 복귀까지의 상한(초) |
| `go_to_goal` | true | false 면 undock 한 자리에서 바로 탐색 |
| `goal_x`, `goal_y` | -3.88, -2.84 | 탐색을 시작할 지도 좌표(웹캠 근처) |
| `dock_front_x`, `dock_front_y` | -1.0, -0.07 | 돌아올 dock 앞 좌표 |

### 터미널 5 — 중단과 확인

추종을 끝내고 dock 시키기:

```bash
ros2 service call /robot4/stop_follow std_srvs/srv/Trigger
```

명령을 친 뒤 로봇이 반응하기까지 6~7 초 걸린다(명령이 서비스를 찾는 시간). 노드가 받은 뒤에는 바로 멈춘다.

보고 싶을 때:

```bash
ros2 topic echo /robot4/amr_state            # IDLE / MOVING / SCANNING / FOLLOWING
ros2 topic echo /robot4/rc_car_target        # RC카까지 거리(distance), 좌우(offset)
ros2 run rqt_image_view rqt_image_view /robot4/rc_car_debug/compressed   # 박스와 거리가 그려진 로봇 카메라 영상
```

`ros2 topic list` 가 `/parameter_events`, `/rosout` 만 보이면 `ros2 daemon stop; ros2 daemon start` 를 하고 두 번쯤 다시 친다.

## 5. VICTUS (`hv-04`)

### 터미널 1 — 고정 웹캠 노드

```bash
ros2 run vision_pkg webcam_node
```

- 웹캠 창이 뜨고 왼쪽 위에 `SEARCHING 0/3` 이 보이면 준비된 것이다.
- 창을 띄울 수 없는 환경이면 `--ros-args -p show_window:=false` 를 붙인다.
- `웹캠을 열 수 없습니다: camera_index=2` 가 나오면 외장 웹캠이 안 꽂혔거나 번호가 다른 것이다. `v4l2-ctl --list-devices` 로 번호를 확인해 `--ros-args -p camera_index:=<번호>` 로 준다.
- 이 노드는 **응답을 한 번 받으면 스스로 끝난다.** 다시 시연하려면 이 명령을 다시 친다.

## 6. 시연

| 순서 | 하는 일 / 일어나는 일 | 걸리는 시간 (10/6 실측) |
|---|---|---|
| 1 | RC카를 몰아 웹캠 시야 안으로 들어온다. 들어와서 1 초쯤 머문다 | 연속 3 프레임(약 0.6 초) 검출되면 요청 |
| 2 | 로봇이 알림음(낮은음→높은음)을 내고 undock 한다 | 약 6 초 |
| 3 | Nav2 가 준비되면 지정 좌표로 이동한다 | 준비 4 초 + 이동 23 초 |
| 4 | 제자리에서 돌며 RC카를 찾는다 | 3~10 초 |
| 5 | RC카를 따라간다. RC카가 멈추면 약 0.8 m 에서 멈춘다 | — |
| 6 | 터미널 5 에서 중단 명령을 친다. 알림음(높은음→낮은음)을 내고 dock 앞으로 돌아와 dock 한다 | 복귀 5~17 초 + dock 22 초 |

RC카를 몰 때:

- 로봇의 최고 속도는 0.2 m/s 다. 천천히 몬다.
- 로봇 쪽으로 다가오지 않는다. 로봇은 후진하지 않고, 0.64 m 안으로 들어오면 거리 값이 끊긴다.
- 로봇 카메라의 좌우 시야는 약 63° 다. 옆으로 빨리 빠지면 놓친다. 놓치면 2 초 뒤 다시 돌며 찾는다.

## 7. 끝내기

1. 로봇이 dock 한 것을 확인한다(터미널 4 에 `상태: IDLE`).
2. 터미널 4, 3 을 `Ctrl+C` 로 끈다. VICTUS 의 웹캠 노드는 이미 끝나 있다.
3. Nav2 와 위치 추정(터미널 1, 2)은 다른 팀원이 쓰는지 확인하고 끈다.

주행 노드를 추종 도중에 `Ctrl+C` 로 끄면 로봇은 그 자리에 멈춘다. 그때는 손으로 dock 시킨다.

```bash
ros2 action send_goal /robot4/dock irobot_create_msgs/action/Dock "{}"
```

dock 이 멀어 실패하면 로봇을 스테이션 앞으로 옮긴 뒤 다시 친다.

## 8. 안 될 때

| 증상 | 원인 | 조치 |
|---|---|---|
| 터미널 3 에 `TF 대기 중` 만 찍힌다 | `/tf` 리매핑이 빠졌다 | 터미널 3 명령을 그대로 다시 친다 |
| 터미널 3 에 undock 뒤에도 `영상 없음` | 로봇 카메라가 안 켜졌거나 Wi-Fi 문제 | `ros2 topic hz /robot4/oakd/stereo/image_raw` 로 약 10 Hz 가 나오는지 본다 |
| 터미널 4 가 `웹캠 검출 신호 대기` 전에 멈춘다 | 로봇의 도킹 상태를 못 받았다 | 로봇 전원과 Wi-Fi, `ros2 topic echo --once /robot4/dock_status` |
| 터미널 4 가 undock 뒤 멈춰 있다 | Nav2 가 켜져 있지 않다 | 터미널 1, 2 를 확인한다 |
| 웹캠 노드가 `서비스를 기다리는 중` 만 반복 | 터미널 4 가 안 떠 있다 | 터미널 4 를 띄운다. 뜨면 자동으로 요청이 간다 |
| RC카가 웹캠에 보이는데 출발하지 않는다 | 신뢰도가 0.85 에 못 미친다 | 웹캠 창의 박스 숫자를 본다. 조명이나 RC카 위치를 바꾼다 |
| 로봇이 RC카 앞에서 방향만 맞추고 안 다가온다 | RC카가 유지 거리(0.8 m)보다 가깝다 | 정상이다. RC카를 멀리 옮긴다 |
| 로봇이 자꾸 돌며 다시 찾는다 | 2 초 넘게 RC카를 못 봤다 | RC카를 로봇 정면 0.8~3 m 에 둔다 |
| 웹캠 없이 출발만 시켜 보고 싶다 | — | `ros2 service call /robot4/rc_car_detected interface_pkg/srv/WebcamDetection "{detected: true}"` |
