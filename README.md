# nav_cctv
2022 Capstone Design (A - AGV)
Create custom layer(CCTV layer) and unify with global costmap.   
Finally, robot will consider CCTV based global obstacles while genenrating global path by global planner. 

## launch
old.launch >> make costmap using pre-hyro method  
new.launch >> make costmap using layered costmap  
dynamic_point.launch >> consist layer which is subscribe and marking dynamic point  
cctv.launch >> consist layer which is subscribe and marking multiple dynamic point  

## Base package
ros-planning navigation package (lower costmap_2d and move_base)  
practice package (our robot package which is include odom, sensor, tf, etc... )  
