# nav_cctv
2022 Capstone Design (A - AGV) 
ROS navigation costmap에 cctv layer를 병합. 

## layer구성
global costmap : static layer, cctv layer, inflation layer
local costmap : obstacle layer, inflation layer


## launch
cctv.launch >> /points topic(cctv영상에 YOLO 알고리즘을 적용하여 얻어낸 동적장애물(사람)의 좌표)을 subscribe하여 navigation costmap에 병합하여 출력

## dynamic reconfigure
마킹반경을 동적재구성이 가능하도록 적용.
move_base/global_costmap/cctv_layer
lethal_radius = 좌표 마킹반경

## Base package
ros-planning navigation package (costmap_2d, move_base)  
practice package (제어부에서 만든 모바일로봇 컨트롤 패키지)  
