#!/usr/bin/env python3

import socketio
import socketio.asgi
import socketio.async_server
import rospy
from std_msgs.msg import  Int32, Float32, String
from sensor_msgs.msg import Image
from ie_communication.msg import  SensorDataMap, TaskData, Param
from ie_communication.srv import camState, robotTask, robotGear, robotGearResponse, taskMessage, taskMessageResponse, mappingOutput, mappingOutputResponse
from cv_bridge import CvBridge
import asyncio
import uvicorn
import cv2
import base64

class ie_API_Server:

    def __init__(self):
        
        self._bridge = CvBridge()
        # Initialize SocketIO with CORS allowed
        self.sio = socketio.async_server.AsyncServer(async_mode='asgi', cors_allowed_origins="*")
        self.app = socketio.asgi.ASGIApp(self.sio)

        # Define publishers and subscribers
        self.mc_pub = rospy.Publisher("manual_controller", Int32, queue_size=10)
        self.cam_sub = rospy.Subscriber("camera_feed", Image, self.run_async_cameraFeedCallback)
        self.cam_qr_sub = rospy.Subscriber("camera_qr_code_feed", Image, self.run_async_cameraQrFeedCallback)
        self.sensors_sub = rospy.Subscriber("sensor_data", SensorDataMap, self.sensorsCallback)
        self.speed_sub = rospy.Subscriber("speed_value", Float32, self.run_async_speedCallback)
        self.map_sub = rospy.Subscriber("map_feed", Image, self.run_async_mapFeedCallback)
        self.process_sub = rospy.Subscriber("ProcessState", String, self.run_async_processState)


        # Define services
        rospy.Service('output', taskMessage, self.run_async_taskFinished)
        rospy.Service('mapping_output', mappingOutput, self.run_async_mappingOutput)
        rospy.Service('gear_changed', robotGear, self.run_async_gearChanged)

        # Register event handlers
        self.sio.on("connect", self.onConnect)
        self.sio.on("disconnect", self.onDisconnect)
        self.sio.on("moveDirection", self.movement)
        self.sio.on("message", self.message)
        self.sio.on("robot_task", self.taskCallback)
        self.sio.on("robot_gear", self.gearCallback)

    def sensorsCallback(self, data):
        pass
    
    def run_async_cameraFeedCallback(self,data):
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.cameraFeedCallback(data))
        except Exception as e:
            rospy.logerr(f"Error before proccessing and emitting camera feed: {e}")
        
    async def cameraFeedCallback(self, data):
        try:
            # Convert ROS Image to OpenCV image using CvBridge
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Encode the OpenCV image as JPEG (or PNG)
            _, buffer = cv2.imencode('.png', cv_image)
            
            # Convert to base64 string for transmission
            data_base64 = base64.b64encode(buffer).decode('utf-8')

            # Emit the encoded image through the socket
            await self.sio.emit("camera_feed", {"image": data_base64})

        except Exception as e:
            rospy.logerr(f"Error processing and emitting camera feed: {e}")

    def run_async_cameraQrFeedCallback(self,data):
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.cameraQrFeedCallback(data))
        except Exception as e:
            rospy.logerr(f"Error before proccessing and emitting camera feed: {e}")
        
    async def cameraQrFeedCallback(self, data):
        try:
            # Convert ROS Image to OpenCV image using CvBridge
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Encode the OpenCV image as JPEG (or PNG)
            _, buffer = cv2.imencode('.png', cv_image)
            
            # Convert to base64 string for transmission
            data_base64 = base64.b64encode(buffer).decode('utf-8')

            # Emit the encoded image through the socket
            await self.sio.emit("camera_qr_feed", {"image": data_base64})

        except Exception as e:
            rospy.logerr(f"Error processing and emitting camera feed: {e}")

    def run_async_speedCallback(self,data):
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.speedCallback(data))
        except Exception as e:
            rospy.logerr(f"Error before proccessing and emitting camera feed: {e}")
        
    async def speedCallback(self, data):
        try:
            await self.sio.emit("speed", {"speed": data.data})
        except Exception as e:
            rospy.logerr(f"Error emitting speed: {e}")

    def run_async_mapFeedCallback(self,data):
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.mapFeedCallback(data))
        except Exception as e:
            rospy.logerr(f"Error before proccessing and emitting map feed: {e}")

    async def mapFeedCallback(self, data):
        try:
            # Convert ROS Image to OpenCV image using CvBridge
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Encode the OpenCV image as JPEG (or PNG)
            _, buffer = cv2.imencode('.png', cv_image)
            
            # Convert to base64 string for transmission
            data_base64 = base64.b64encode(buffer).decode('utf-8')

            # Emit the encoded image through the socket
            await self.sio.emit("map_feed", {"image": data_base64})

        except Exception as e:
            rospy.logerr(f"Error processing and emitting map feed: {e}")

    async def gearCallback(self, sid, data):
        print(data)
        rospy.wait_for_service('change_gear')
        robot_gear = rospy.ServiceProxy('change_gear', robotGear)
        robot_gear.wait_for_service(10)
        try:
            response = robot_gear(data["gear"])
            if response.message:
                print("changed")
            else:
                print("Failed")
            
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    async def taskCallback(self, sid, task):
        t = TaskData()
        t.task_name = task['task']["task_name"]
        t.params =  [Param(zone=key,type=value) for key, value in task['task']["params"].items()]
        rospy.wait_for_service('robot_task')
        robot_task = rospy.ServiceProxy('robot_task', robotTask)
        robot_task.wait_for_service(10)
        try:
            response = robot_task(t)
            print(response)
            await self.sio.emit("task_response", {"response": response.message}, to=sid)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))   

    def run_async_taskFinished(self,output):
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.taskFinished(output))
        except Exception as e:
            rospy.logerr(f"Error before proccessing and emitting taskFinished: {e}")

        return taskMessageResponse(True)
    
    async def taskFinished(self, output):
        await self.sio.emit("task_response", {"response": output.message})

    def run_async_mappingOutput(self,output):
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.mappingOutput(output))
        except Exception as e:
            rospy.logerr(f"Error before proccessing and emitting taskFinished: {e}")

        return mappingOutputResponse(True)
    
    async def mappingOutput(self, output):
        print(output)
        data = [{"location" :kv.location, "x": kv.x, "y":kv.y} for kv in output.locations]
        print(data)
        await self.sio.emit("mapping_output", {"locations": data})

    def run_async_processState(self,state):
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.processState(state))
        except Exception as e:
            rospy.logerr(f"Error before proccessing and emitting processState: {e}")
    
    async def processState(self, state):
        try:
            await self.sio.emit("processState", {"state": state.data})

        except Exception as e:
            rospy.logerr(f"Error processing and emitting map feed: {e}")

    async def onConnect(self, sid, environ):
        print(f"client {sid} connected")
    
    async def onDisconnect(self, sid):
        print(f"client {sid} disconnected")

    async def movement(self, sid, direction):
        print(direction["direction"])
        if direction["direction"] == "UP" :
            self.mc_pub.publish(1)
        elif direction["direction"] == "DOWN" :
            self.mc_pub.publish(2)
        elif direction["direction"] == "LEFT" :
            self.mc_pub.publish(4)
        elif direction["direction"] == "RIGHT":
            self.mc_pub.publish(3)
        elif direction["direction"] == "STOP":
            self.mc_pub.publish(5)

    def connect(self):
        try:
            # Start the eventlet WSGI server
            uvicorn.run(self.app, host='0.0.0.0', port=5000)
        except KeyboardInterrupt:
            rospy.loginfo("Server shutting down gracefully...")
            self._stop()
        except Exception as e:
            rospy.logerr(f"Unexpected server error: {e}")
            self._stop()

    async def message(self, sid, message):
        print(f"client {sid} : {message}")
    
    def _stop(self):
        rospy.loginfo("Server shutting down...")
        self.sio.stop()  # Stop Socket.IO
    
    def run_async_gearChanged(self, gear):
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.gearChanged(gear))
            return robotGearResponse(True)
        except Exception as e:
            rospy.logerr(f"Error before proccessing and emitting gearChanged: {e}")

            return robotGearResponse(False)
    
    async def gearChanged(self, gear):
        await self.sio.emit("gear_state", {"gear": gear.state})

if __name__ == '__main__':
    from threading import Thread
    
    Thread(target=lambda: rospy.init_node('ie_Api', disable_signals=True)).start()
    server = ie_API_Server()
    server.connect()
    rospy.spin()
