1. 
git clone https://github.com/kdh4970/nav_cctv.git

2.
nav_cctv/movs_base_add/include에 있는 헤더파일 전부 navigation/move_base/include로 이동.
nav_cctv/movs_base_add/src에 있는 cpp파일 전부 navigation/move_base/src로 이동.
(move_base_node.cpp는 교체)

3. 
nav_cctv/src에 있는 cctv_layer.cpp, dynamic_layer.cpp, listener.cpp 에서
마지막 #include의 cpp파일 경로를 본인 경로로 수정

4. 
패키지 빌드 


5.
사용할 터틀봇 네비게이션 런치 파일에서 move_base 의 costmap 파라미터부분 변경해야 함.

5-1.
nav_cctv/launch/cctv.launch 파일의 point publisher 부분을 복사하여 사용할 런치파일의 move_base 보다 위에 붙여넣기.

5-2.
nav_cctv/launch/cctv.launch 파일의 move_base 부분 중 순서대로 4개 복붙(costmap 관련 파라미터)

6.
실행 

7.
#오류 발생시 각 nav_cctv의 new_costmap_common_param, cctv_global_params, new_local_params를 열고, global_frame, robot_base_frame, observation source, topic 파라미터를 터틀봇의 파라미터로 교체.
