## **in this branch you will find my  contribution to the capstone project**

i divided my work into two folders :

* the first one (GUI) : in this folder we find everything related to the gui and instruction for setup
* the second one (ROS) : in that folder we find a ros package i created which contains 5nodes and 3 classes:
  * ie_api : this node handle the communication between ros and the web application
  * movementController : this node update the robot movement according to what user desires to use through the GUI
  * cameraController : this node manage all of the camera found on the robot and conver them to images to send to the web   application
  * task: this class manages the line following system, with detection of junction and choice of side to continue and also qr code detection, it is also a parent class for task like mapping or carrying
  * mapping : this class manages the entire mapping process
  * carrying : this class is used to handle carrying process which involves moving the robot to a loading station, load a charge, move to a unload station, unload the charge
  * processController : this is a process manager,  this node handle every process which can occur, plan them
  * mapProcess : this node handle everything related to map, and lidar informations
