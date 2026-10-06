# AMR 상태
# 토픽: /robot4/amr_state   (navi_pkg -> vision_pkg, 1 Hz + 바뀔 때)

string IDLE=IDLE             # 검출 신호 대기
string MOVING=MOVING         # 지정 좌표로 이동 중
string SCANNING=SCANNING     # scan_motion 으로 RC카 탐색 중
string FOLLOWING=FOLLOWING   # RC카 추종 중 (거리 유지로 멈춰 있는 것도 포함)

string state
