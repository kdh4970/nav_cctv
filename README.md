# CCTV_Layer_ROS
2022 Capstone Design (A - AGV)
This package purpose is to use deepsort-yolov4 on ROS.   
Finally, /YOLO node publish topic(/points).

## Development Environment

OS : Ubuntu 18.04 LTS  
Meta OS : ROS1-melodic  
GPU : NVIDIA GTX 1080 Ti  
NVIDIA Graphic Driver version : 470  
CUDA version : 11.4  
CUDNN version : 8.2.4  
Python Interpreter version : Python 3.6.9  
OpenCV version : 4.1.1.26  
Tensorflow version : 2.3.0  
USB mono camera : PWC 500 (2EA)  

## Initial Settings ( Do only first time )
First. Download yolov4-tiny.weights into 'data' folder.  
(https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights)  

Second. Run save_model.py on terminal.
```
# save yolov4-tiny model
python save_model.py --weights ./data/yolov4-tiny.weights --output ./checkpoints/yolov4-tiny-416 --model yolov4 --tiny
```

Third. Add PYTHONPATH  
You should add 'msg' folder to PYTHONPATH.  

Fourth. Set flags.define paths ( main script - track0515.py )  
Change weights, video, output path.  

Fifth. Set model path (main script - tracker0515.py) 
After find below line, change between * and *.
```python
model_filename = '/home/*sh/catkin_ws/src/ros_yolov4*/src/model_data/mars-small128.pb'
```
Final. Do catkin_make.  


## How to Use  
First. Connect two usbwebcam.  
Second. Execute launch file.  
```
$ roslaunch 'pkg name' detect.launch
```




## Base package
TheAIGuysCode - yolov4-deepsort (https://github.com/theAIGuysCode/yolov4-deepsort)
