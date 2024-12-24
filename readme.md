## **in this branch you will find my  contribution to the capstone project**

i divided my work into two folders :

* the first one (GUI) : in this folder we find everything related to the gui and instruction for setup
* the second one (ROS) : in that folder we find a ros package i created which contains 3 nodes :
  * ie_api : this node handle the communication between ros and the web application
  * movementController : this node update the robot movement according to what user desires to use through the GUI
  * cameraController : this node manage all of the camera found on the robot and conver them to images to send to the web   application
