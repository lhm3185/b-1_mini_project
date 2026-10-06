# 고정 웹캠의 RC카 검출 신호
# 토픽: /webcam/rc_car_detected   (vision_pkg -> navi_pkg, 5 Hz 로 계속 발행)

std_msgs/Header header   # stamp = 영상이 찍힌 시각
bool detected            # RC카가 보이면 true
