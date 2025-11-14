import serial
import time
import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import DistanceSensor
import paho.mqtt.client as mqtt
import json

try:
    ser = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=0.5)
    time.sleep(2)  # UART initialize
    print("✅ Serial port open on /dev/ttyAMA0 (9600 baud)")
except Exception as e:
    print(f"❌ Failed to open serial port: {e}")
    sys.exit(1)
leftstep = 100
rightstep = 100
aroundstep = 100
forwardstep = 100

sensor1=0
sensor2=0
sensor3=0

xx, yy = 0, 0
prev_xx, prev_yy = 0, 0
tarx,tary=0,0
facing = 0
phase = 1
a = 0
msgfromros = ''

latest_command = ''
latest_steps = 0
latest_dir = ''
state=''
distancetowall = 12

BROKER = "broker.hivemq.com"
PORT = 1883

def on_connect(client, userdata, flags, rc):
    print("✅ MQTT Connected with result code", rc)
    client.subscribe("robot/tracking_data")
    client.subscribe("dir")
    print("📡 Subscribed to topics: step, dir")

def on_message(client, userdata, msg):
    global latest_command, latest_steps, latest_dir, xx, yy, state
    payload = msg.payload.decode().strip()
    topic = msg.topic
    #print(f"📩 Received on '{topic}': {payload}")

    if topic == "robot/tracking_data":
        # Split by commas first -> ["state/0", "xx/2", "yy/3"]
        # print(payload)
        datas=json.loads(payload)
        xx=datas["grid_x"]
        yy=datas["grid_y"]
        state = datas["state"]
        if xx != prev_xx or yy != prev_yy:
            print("stop")
            user_input = "stop"
            ser.write((user_input + "\n").encode('utf-8'))


    elif topic == "dir":
        latest_dir = payload.strip()



client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT, 60)
client.loop_start()



# from geometry_msgs.msg import Pose2D as POSE_2D  # example substitute

# class GridSubscriber(Node):
#     def __init__(self):
#         super().__init__('grid_subscriber')
#         self.subscription = self.create_subscription(
#             POSE_2D,
#             'grid_location',
#             self.listener_callback,
#             10
#         )
#         self.subscription

#     def listener_callback(self, msg):
#         global xx, yy
#         xx = msg.x
#         yy = msg.y
#         self.get_logger().info(f"🔹ROS Received grid_location: x={xx}, y={yy}")

# def ros2_thread():
#     rclpy.init()
#     node = GridSubscriber()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# t_ros = threading.Thread(target=ros2_thread)
# t_ros.start()
    
running=True
while running:
    print(f"Grid :[{xx},{yy}] , prev:[{prev_xx},{prev_yy}], state: {state}]")
 
    print(phase)
    if latest_dir == "start" :
        phase = 1

    if latest_dir== "phase0":
        phase=0
        print(f"Now your phase is : {phase}")
    if phase == 0:
        yaw=ser.readline().decode('utf-8').strip()
        print("yaw=",yaw)



    if phase ==1 :  # mapping phase
        # เลี้ยวขวา
        response = ser.readline().decode('utf-8').strip()

        if response=="right":
            print(response)
            client.publish("state","r")#right

        # กลับรถ
        elif response == "around":
            print(response)
            client.publish("state","b")#turn around

        # ตรงไป
        elif response == "forwardmap":
            print(response)
            client.publish("state","s")#Forward

        # เลี้ยวซ้าย
        elif response == "left":
            print(response)
            client.publish("state","l") #turn left
    if phase ==2:
        response = ser.readline().decode('utf-8').strip()
        if response == "ask":
            print("ask")
            client.publish("state","ask")
            response = ''
            if latest_dir =="right":
                print(f"it' s right")
                user_input = "right" + str(0)
                ser.write((user_input + "\n").encode('utf-8'))
            elif latest_dir =="left":
                print("it' s left")
                user_input = "left" + str(0)
                ser.write((user_input + "\n").encode('utf-8'))
            elif latest_dir =="forward":
                print("it' s forward")
                user_input = "forward" + str(0)
                ser.write((user_input + "\n").encode('utf-8'))

    prev_xx = xx
    prev_yy= yy

client.loop_stop()
client.disconnect()
