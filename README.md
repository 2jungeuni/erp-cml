Code for control ERP-42

코드 사용 방법(우분투 + vscode 사용시)

1. 터미널에서 workspace 및 src 폴더 생성 (ex) mkdir -p erp_ws/src)
2. vscode 실행
3. 왼쪽 창의 세번쩨 항목 '소스 제어' 선택 후 '리포지토리 복제' 누르기
4. github 로그인(아마) -> erp-cml 선택
5. erp_ws/src 폴더 안에서 레포지토리 위치시키기
6. (branch 변경) 왼쪽 아래의 'main' 클릭 -> sjlee 선택
7. 터미널의 erp_ws폴더에서 catkin_make로 빌드 후 코드 실행하기


---
실행하기 (따로 roscore를 실행해두면 편함)

카메라 실행
- `roslaunch realsense2_camera rs_camera.launch color_width:=640 color_height:=360 color_fps:=30 depth_width:=640 depth_height:=360 depth_fps:=30`
- 또는 `roslaunch realsense2_camera rs_camera.launch` 

--> 이제 `rosrun camera_data turn_on_camera.py`가 답이다.

카메라 이미지 처리
- `rosrun camera_data image_analysis2.py`  -> 7층 주행
- `rosrun camera_data main_ROS.py` -> 차선인식
- `rosrun camera_data main_ROS_prs.py` -> ApproximateTimeSynchronizer 사용 안한 버전(depth가 늦게 들어와도 연산 가능)

reference point를 기반으로 차량 제어(pure pursuit)
- `rosrun testdrive final_testdrive.py`

ERP 통신
- `rosrun erp_com receiver.py`
- `rosrun erp_com sender.py`

---
* 5/15, 5/30, 6/17 날짜의 rosbag 주행데이터는 notion에 업로드 함(https://www.notion.so/rml-erp42/Lab-Driving-Test-8290b640e4624a1090318b775625869a).
