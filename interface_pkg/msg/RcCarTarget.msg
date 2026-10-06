# AMR 카메라(OAK-D)로 본 RC카까지의 거리와 방향 (depth 로 계산)
# 토픽: /robot4/rc_car_target   (vision_pkg -> navi_pkg, RC카가 보일 때만 발행, 초당 약 10 번)
# 1.0 초 넘게 오지 않으면 놓친 것으로 본다.

std_msgs/Header header   # stamp = 영상이 찍힌 시각, frame_id = "base_link"
float32 distance         # 전방 거리 (m), 로봇 중심 기준
float32 offset           # 좌우 치우침 (m). 왼쪽 +, 오른쪽 -
