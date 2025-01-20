the urdf dimention is close to the project 85cm * 90cm * 50cm , 
i added the lifting system 
to change the position run this code:
```bash
rostopic pub /position_joint_controller/command std_msgs/Float64 "data: 1.0"
```
or publish on /position_joint_controller/command between 0 and 1 ,
for any furthur changes pleas contact me
##note:
you might need to change couple of codes depending on where you save it , dont forget to catkin make .
