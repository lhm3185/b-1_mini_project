# 실행 가이드

RC카가 고정 웹캠에 보이면 AMR 이 출발해 RC카를 찾아 따라가고, 중단을 요청하면 dock 으로 돌아오는 시나리오를 돌리는 순서다.
**PC 한 대, 터미널 두 개**(+ 중단 명령용 하나)로 띄운다. 토픽·서비스의 뜻은 [interfaces.md](interfaces.md) 를 본다.

## 0. 한눈에 보기

| 터미널 | 명령 | 띄우는 것 | 준비됐다는 표시 |
|---|---|---|---|
| 1 | `ros2 launch navi_pkg navi.launch.py` | 위치 추정, Nav2, RViz, 주행 노드 `follow_car` | `웹캠 검출 신호 대기 (서비스 rc_car_detected)` |
| 2 | `ros2 launch vision_pkg vision.launch.py` | AMR 캠 `amr_cam`, 고정 웹캠 `webcam_node` | AMR 캠: `영상 없음 (도킹 중이면 카메라가 꺼져 있다)` / 웹캠: 창이 뜨고 `SEARCHING 0/3` |
| 3 | `ros2 service call /robot4/stop_follow std_srvs/srv/Trigger` | 추종 중단 | — |

터미널 1 을 먼저, "웹캠 검출 신호 대기" 가 찍힌 뒤 터미널 2 를 띄운다. 웹캠 노드는 RC카가 보이는 순간 바로 출발 신호를 보낸다.

## 1. 시작 전 확인

- 로봇이 **도킹 스테이션에 올라가 있다.**
- PC 가 Wi-Fi `turtle08` 에 붙어 있다.
- 고정 웹캠이 이 PC 에 꽂혀 있다. `ls /dev/video*` 로 번호를 본다(노트북 내장 카메라가 0, 1 이면 외장은 2).
- RC카는 **웹캠 시야 밖**에 둔다.
- 지도 파일이 `~/maps/my_map.yaml` 에 있다(다른 곳이면 `map:=` 로 준다).
- AMR 캠 모델이 `vision_pkg/models/amrcam_yolo26n.pt` 에 있다.

## 2. 코드 받기와 빌드 (처음 한 번)

```bash
mkdir -p ~/amr_cam_test_ws/src && cd ~/amr_cam_test_ws/src
git clone -b feature/rc_car_follow https://github.com/lhm3185/b-1_mini_project.git
cp <amrcam_yolo26n.pt 가 있는 곳>/amrcam_yolo26n.pt b-1_mini_project/vision_pkg/models/
cd ~/amr_cam_test_ws
colcon build --symlink-install
```

이미 받아 둔 폴더면 `git pull` 뒤 다시 빌드한다. `which colcon` 이 `~/venvs/rokey_venv/...` 여야 한다(시스템 colcon 으로 빌드하면 노드가 venv 의 ultralytics 를 찾지 못한다).
한 폴더에서 `--symlink-install` 을 쓰는 빌드와 안 쓰는 빌드를 섞으면 `interface_pkg` 빌드가 실패한다. 그때는 `build/interface_pkg` 와 `install/interface_pkg` 를 지우고 다시 빌드한다.

## 3. 터미널마다 먼저 칠 것

```bash
source ~/venvs/rokey_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/turtlebot4_ws/install/setup.bash
source /etc/turtlebot4_discovery/setup.bash
source ~/amr_cam_test_ws/install/setup.bash     # 반드시 마지막에
```

`.bashrc` 가 앞의 네 줄을 이미 하고 있으면 마지막 줄만 치면 된다. 마지막 줄이 맨 뒤에 와야 한다(다른 워크스페이스의 옛 `interface_pkg` 가 먼저 잡히면 타입이 맞지 않는다).

## 4. 터미널 1 — AMR

```bash
ros2 launch navi_pkg navi.launch.py
```

- 위치 추정, Nav2, RViz, 주행 노드가 한 번에 뜬다. `웹캠 검출 신호 대기 (서비스 rc_car_detected)` 가 찍히면 준비된 것이다. 로봇은 아직 움직이지 않는다.
- Nav2 가 이미 다른 터미널에 떠 있으면 `nav2:=false` 를 붙여 주행 노드(와 RViz)만 띄운다.
- 한 번 시연이 끝나 dock 하면 다시 `웹캠 검출 신호 대기` 로 돌아간다. 이 터미널은 계속 둔다.

| 인자 | 기본값 | 뜻 |
|---|---|---|
| `map` | `~/maps/my_map.yaml` | 지도 파일 |
| `nav2` | true | false 면 위치 추정·Nav2 를 띄우지 않는다 |
| `rviz` | true | RViz |
| `follow_sec` | 600.0 | 출발부터 이 시간이 지나면 스스로 복귀(초) |

## 5. 터미널 2 — 비전

```bash
ros2 launch vision_pkg vision.launch.py
```

- **AMR 캠:** `amr_cam 시작: conf=0.85, ...` 다음에 `영상 없음 (도킹 중이면 카메라가 꺼져 있다)` 가 5 초마다 찍히면 정상이다. 로봇이 undock 하면 `영상 수신 중` 으로 바뀌고, 박스와 거리가 그려진 창("AMR Cam RC Car Detection")이 뜬다.
- **고정 웹캠:** 창("Webcam RC Car Detection")이 뜨고 왼쪽 위에 `SEARCHING 0/3` 이 보인다.
- 웹캠 번호가 2 가 아니면 `camera_index:=<번호>`, 웹캠이 다른 PC 에 있으면 여기서는 `webcam:=false` 로 띄우고 그 PC 에서 `ros2 run vision_pkg webcam_node` 를 친다.
- 웹캠 노드는 **응답을 한 번 받으면 스스로 끝난다.** 다시 시연하려면 이 터미널을 `Ctrl+C` 로 끄고 다시 띄운다.

## 6. 시연

| 순서 | 하는 일 / 일어나는 일 | 걸리는 시간 (10/6 실측) |
|---|---|---|
| 1 | RC카를 몰아 웹캠 시야 안으로 들어온다. 들어와서 1 초쯤 머문다 | 연속 3 프레임(약 0.6 초) 검출되면 요청 |
| 2 | 로봇이 알림음(낮은음→높은음)을 내고 undock 한다 | 약 6 초 |
| 3 | Nav2 가 준비되면 지정 좌표로 이동한다 | 준비 4 초 + 이동 23 초 |
| 4 | 제자리에서 돌며 RC카를 찾는다 | 3~10 초 |
| 5 | RC카를 따라간다. RC카가 멈추면 약 0.8 m 에서 멈춘다 | — |
| 6 | 터미널 3 에서 중단 명령을 친다. 알림음(높은음→낮은음)을 내고 dock 앞으로 돌아와 dock 한다 | 복귀 5~17 초 + dock 22 초 |

중단 명령(터미널 3):

```bash
ros2 service call /robot4/stop_follow std_srvs/srv/Trigger
```

명령을 친 뒤 로봇이 반응하기까지 6~7 초 걸린다(명령이 서비스를 찾는 시간). 노드가 받은 뒤에는 바로 멈춘다.

RC카를 몰 때:

- 로봇의 최고 속도는 0.2 m/s 다. 천천히 몬다.
- 로봇 쪽으로 다가오지 않는다. 로봇은 후진하지 않고, 0.64 m 안으로 들어오면 거리 값이 끊긴다.
- 로봇 카메라의 좌우 시야는 약 63° 다. 옆으로 빨리 빠지면 놓친다. 놓치면 2 초 뒤 다시 돌며 찾는다.

## 7. 끝내기

1. 로봇이 dock 한 것을 확인한다(터미널 1 에 `상태: IDLE`).
2. 터미널 2, 1 을 `Ctrl+C` 로 끈다.

주행 노드를 추종 도중에 `Ctrl+C` 로 끄면 로봇은 그 자리에 멈춘다. 그때는 손으로 dock 시킨다.

```bash
ros2 action send_goal /robot4/dock irobot_create_msgs/action/Dock "{}"
```

## 8. 안 될 때

| 증상 | 원인 | 조치 |
|---|---|---|
| AMR 캠에 undock 뒤에도 `영상 없음` | 로봇 카메라가 안 켜졌거나 Wi-Fi 문제 | `ros2 topic hz /robot4/oakd/stereo/image_raw` 로 약 10 Hz 가 나오는지 본다 |
| AMR 캠에 `TF 대기 중` 만 찍힌다 | 런치가 아니라 `ros2 run` 으로 띄우면서 `/tf` 리매핑을 뺐다 | 런치로 띄운다 |
| AMR 캠이 모델 파일을 못 찾는다 | `vision_pkg/models/amrcam_yolo26n.pt` 가 없다 | 2장대로 복사하고 다시 빌드 |
| 터미널 1 이 undock 뒤 멈춰 있다 | Nav2 가 아직 준비되지 않았다 | 잠시 기다린다. 계속이면 터미널 1 의 Nav2 오류를 본다 |
| 웹캠 노드가 `서비스를 기다리는 중` 만 반복 | 터미널 1 의 주행 노드가 안 떠 있다 | 터미널 1 을 확인한다. 뜨면 자동으로 요청이 간다 |
| `웹캠을 열 수 없습니다: camera_index=2` | 웹캠이 안 꽂혔거나 번호가 다르다 | `ls /dev/video*` 로 확인해 `camera_index:=<번호>` |
| RC카가 웹캠에 보이는데 출발하지 않는다 | 신뢰도가 0.85 에 못 미친다 | 웹캠 창의 박스 숫자를 본다. 조명이나 RC카 위치를 바꾼다 |
| 로봇이 RC카 앞에서 방향만 맞추고 안 다가온다 | RC카가 유지 거리(0.8 m)보다 가깝다 | 정상이다. RC카를 멀리 옮긴다 |
| `ros2 topic list` 가 거의 비어 있다 | daemon 이 오래됐다 | `ros2 daemon stop; ros2 daemon start` 뒤 두 번쯤 다시 친다 |
| 웹캠 없이 출발만 시켜 보고 싶다 | — | `ros2 service call /robot4/rc_car_detected interface_pkg/srv/WebcamDetection "{detected: true}"` |
