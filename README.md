# tms-perception
![ros-tms](https://avatars2.githubusercontent.com/u/8273459?v=3&s=200)  
Perception packages for [irvs/ros_tms](https://github.com/irvs/ros_tms).  

This is a fork from an original work from Kazuto Nakashima, Kyushu University. 

## tms_ss_rcnn
ROS-based client/server nodes to detect multiple objects from an image. Informations about
detected objects are written to the ROS-TMS Database.
* Object detection server wrapping the [Faster R-CNN](https://github.com/rbgirshick/py-faster-rcnn) and sending proper
data (such as recording date and score) to the ROS-TMS Dabatase.
* Android app to stream camera images, displaying useful informations based on data within the ROS-TMS Database.

**Messaging through ROS service**  
* Image detection 
```sh
# Request: An image compressed to jpeg format
sensor_msgs/CompressedImage image
```
```sh
# Response: Arrays contains a class, score and region
tms_ss_rcnn/object[] objects
```
* Communication with the Database
```sh
# A standardized ROS-TMS Database entry, containing various informations 
tms_db_msg/TmsdbStamped obj_info
```


**Demo**  
Install the app to any Android device and run the server. 
Please refer to this [link](https://github.com/irvs/ros_tms/wiki/how-to-configure-rosjava-apps-with-gradle) for installation of rosjava apps.
```sh
# CPU
$ rosrun tms_ss_rcnn faster_rcnn.py --cpu
# GPU
$ rosrun tms_ss_rcnn faster_rcnn_gpu.py 
# GPU with faster processing time
$ rosrun tms_ss_rcnn faster_rcnn_gpu.py --net zf 
```

## tms_ss_cnn
Simple image recognition nodes

**Demo**  
```sh
$ rosrun tms_ss_cnn server_caffe.py
```

```sh
$ rosrun tms_ss_cnn server_chainer.py
```
