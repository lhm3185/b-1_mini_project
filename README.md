# b-1_mini_project

ROKEY 부트캠프 미니 프로젝트. 고정 웹캠이 RC카를 발견하면 AMR(TurtleBot4)이 지정 좌표로 가서 RC카를 찾고, 일정 거리를 유지하며 따라간다.

| 패키지 | 역할 |
|---|---|
| `vision_pkg` | 고정 웹캠 RC카 검출, AMR 카메라(OAK-D)로 RC카 위치 산출 |
| `navi_pkg` | 검출 신호 수신, 지정 좌표 이동, scan_motion, RC카 추종 |

패키지 사이의 토픽 약속은 **[docs/interfaces.md](docs/interfaces.md)** 에 있다. 구현 전에 먼저 읽는다.

## 받기 · 빌드

강의 환경(`rokey_venv`, `~/rokey_ws`) 기준이다.

```bash
cd ~/rokey_ws/src
git clone https://github.com/lhm3185/b-1_mini_project.git
cd ~/rokey_ws
colcon build --symlink-install --packages-select navi_pkg vision_pkg
source install/setup.bash
```

브랜치를 따로 만들지 않고 `main` 에 바로 올린다. 올리기 전에 `git pull` 을 먼저 한다.
