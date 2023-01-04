# ompl ReedsSheppCurve rviz   
creating a Reeds-Shepp curve with ompl  
set start and end pose with rviz  
show result in path and marker array  

## how to run  
```
roscore
rosrun omplrscurve omplrscurve
roscd omplrscurve/rviz
rviz -d myrviz.rviz
```
set Start pose with 2D Pose Estimate  
set End pose with 2D Nav Goal  
<img src = "./img/set.png" width="60%" height="100%">   

## Dependencies  
ubuntu 18.04, ROS1 melodic  
```
sudo apt install libompl-dev  
```

## result  
<img src = "./img/example.png" width="50%" height="100%">   
<img src = "./img/example2.png" width="50%" height="100%">   
