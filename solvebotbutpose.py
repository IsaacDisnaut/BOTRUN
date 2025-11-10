import serial
import time
import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import DistanceSensor
import paho.mqtt.client as mqtt

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

xx, yy = 2, 3
prev_xx, prev_yy = xx, yy
tarx,tary=0,0
facing = 0
phase = 0
a = 0
msgfromros = ''

latest_command = ''
latest_steps = 0
latest_dir = ''
state=''
distancetowall = 12

sensor_raw1 = DistanceSensor(echo=27, trigger=17)  # left
sensor_raw2 = DistanceSensor(echo=24, trigger=23)  # front
sensor_raw3 = DistanceSensor(echo=6, trigger=5)    # right
sensor1 = 0 # left
sensor2 = 0   # front
sensor3 = 0   # right

BROKER = "test.mosquitto.org"
PORT = 1883

def on_connect(client, userdata, flags, rc):
    print("✅ MQTT Connected with result code", rc)
    client.subscribe("step")
    client.subscribe("dir")
    print("📡 Subscribed to topics: step, dir")

def on_message(client, userdata, msg):
    global latest_command, latest_steps, latest_dir, xx, yy, state
    payload = msg.payload.decode().strip()
    topic = msg.topic
    #print(f"📩 Received on '{topic}': {payload}")

    if topic == "step":
        # Split by commas first -> ["state/0", "xx/2", "yy/3"]
        parts = payload.split(",")
        #print("part:",parts)
        for part in parts:
            if "/" not in part:
                print(f"⚠️ Skipping invalid segment: {part}")
                continue

            key, value_str = part.split("/", 1)
            try:
                value = int(value_str)
            except ValueError:
                print(f"⚠️ Invalid number for {key}: {value_str}")
                continue

            if key == "state":
                state = value
            elif key == "xx":
                xx = value
            elif key == "yy":
                yy = value
            # else:
            #     print(f"⚠️ Unknown key: {key}")

        #print(f"✅ Parsed → state={state}, xx={xx}, yy={yy}")

    elif topic == "dir":
        latest_dir = payload.strip()


   


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT, 60)
client.loop_start()

xx, yy = 0, 0  # current grid position


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

def forwardmap():
    global prev_xx,prev_yy,xx,yy
    sensor2dummy=''
    print("forwarding")
    user_input = "forwardmap/" + str(forwardstep)
    ser.write((user_input + "\n").encode('utf-8'))
    response = ""
    while response != "forwardmap":
        sensor2dummy = sensor_raw2.distance*100  # front

        print(f"wall {sensor2dummy} ")
        if sensor2dummy < distancetowall:
            user_input = "stop/" 
            ser.write((user_input + "\n").encode('utf-8'))
            break
        print(f"x= {xx} y= {yy} prev_xx={prev_xx} prev_yy={prev_yy}")
        if xx != prev_xx or yy != prev_yy:
            user_input = "stop/" 
            ser.write((user_input + "\n").encode('utf-8'))
        response = ser.readline().decode('utf-8').strip()
    print("forward")
    sensor1 = sensor_raw1.distance*100  # left
    sensor2 = sensor_raw2.distance*100  # front
    sensor3 = sensor_raw3.distance*100  # right
    print(f"Left: {sensor1},Front: {sensor2},Right: {sensor3}")
    time.sleep(1)
facingdum = ''
go = ''
fin=""

def forward():
    user_input = "forward/" + str(forwardstep)
    ser.write((user_input + "\n").encode('utf-8'))
    print("forward")
    response = ""
    while response == "":
        response = ser.readline().decode('utf-8').strip()
    
    
def turn_left():
    global facing
    user_input = "left/" + str(leftstep)
    ser.write((user_input + "\n").encode('utf-8'))
    print("left")

    response = ""
    while response == "":
        response = ser.readline().decode('utf-8').strip()
    time.sleep(1)
    forwardmap()
    facing = (facing - 90)%360
    sensor1 = sensor_raw1.distance*100  # left
    sensor2 = sensor_raw2.distance*100  # front
    sensor3 = sensor_raw3.distance*100  # right
    print(f"Left: {sensor1},Front: {sensor2},Right: {sensor3}")
    time.sleep(1)

def turn_right():
    
    global facing
    user_input = "right/" + str(leftstep)
    ser.write((user_input + "\n").encode('utf-8'))
    print("right")  
    response = ""
    while response == "":
        response = ser.readline().decode('utf-8').strip()
    time.sleep(1)
    forwardmap()
    facing = (facing + 90)%360
    sensor1 = sensor_raw1.distance*100  # left
    sensor2 = sensor_raw2.distance*100  # front
    sensor3 = sensor_raw3.distance*100  # right
    print(f"Left: {sensor1},Front: {sensor2},Right: {sensor3}")
    print("righted") 
    time.sleep(1)

def turn_Around():
    global facing
    user_input = "around/" + str(aroundstep)
    ser.write((user_input + "\n").encode('utf-8'))
    print("b")

    response = ""
    while response == "":
        response = ser.readline().decode('utf-8').strip()
    
    time.sleep(1)
    forwardmap()
    facing = (facing + 180)%360
    sensor1 = sensor_raw1.distance*100  # left
    sensor2 = sensor_raw2.distance*100  # front
    sensor3 = sensor_raw3.distance*100  # right
    print(f"Left: {sensor1},Front: {sensor2},Right: {sensor3}")
    time.sleep(1)

def check_exit():
    if yy ==0:
        if facing==0:
            if sensor1 >= distancetowall: #left
                state=f"final/{xx}/{yy}/{facing}"
                client.publish("target",state)
                facingdum=facing
                tarx=xx
                tary=yy
                go="left"
                print(state,"xx,yy,facing")
            if sensor3 >= distancetowall:
                turn_right()
                client.publish("state","r")
            elif sensor2 >= distancetowall:
                forwardmap()
                client.publish("state","s")
            else:
                turn_Around()
                client.publish("state","b")
        if facing == -90 or facing ==270: #forward
            if sensor2 >= distancetowall:
                state=f"final/{xx}/{yy}/{facing}"
                client.publish("target",state)
                facingdum=facing
                tarx=xx
                tary=yy
                go = "forward"
                print(state,xx,yy,facing)
            
            if sensor3 >= distancetowall:
                turn_right()
                client.publish("state","r")
            elif sensor1 >= distancetowall:
                turn_left()
                client.publish("state","l")
            else:
                turn_Around()
                client.publish("state","b")

        if abs(facing) == 180: #right
            if sensor3 >= distancetowall:
                state=f"final/{xx}/{yy}/{facing}"
                client.publish("target",state)
                facingdum=facing
                tarx=xx
                tary=yy
                go = "right"
                print(state,xx,yy,facing)
            if sensor2 >= distancetowall:
                forwardmap()
                client.publish("state","s")
            elif sensor1 >= distancetowall:
                turn_left()
                client.publish("state","l")
            else:
                turn_Around()
                client.publish("state","b")

    elif yy ==9:
        if facing==0:
            if sensor3 >= distancetowall: #right
                state=f"final/{xx}/{yy}/{facing}"
                client.publish("target",state)
                facingdum=facing
                tarx=xx
                tary=yy
                go = "forward"
                print(state,xx,yy,facing)
            if sensor2 >= distancetowall:
                turn_right()
                client.publish("state","r")
            elif sensor1 >= distancetowall:
                turn_left()
                client.publish("state","l")
            else:
                turn_Around()
                client.publish("state","b")

        if abs(facing) == 180:
            if sensor1 >= distancetowall: #left
                state=f"final/{xx}/{yy}/{facing}"
                client.publish("target",state)
                facingdum=facing
                tarx=xx
                tary=yy
                go="left"
                print(state,xx,yy,facing)
            if sensor3 >= distancetowall:
                turn_right()
                client.publish("state","r")
            elif sensor2 >= distancetowall:
                forwardmap()
                client.publish("state","s")
            else:
                turn_Around()
                client.publish("state","b")

        if facing == 90 or facing == -270: #forward
            if sensor2 >= distancetowall:
                state=f"final/{xx}/{yy}/{facing}"
                client.publish("target",state)
                facingdum=facing
                tarx=xx
                tary=yy
                go = "right"
                print(state,xx,yy,facing)
            if sensor3 >= distancetowall:
                turn_right()
                client.publish("state","r")
            elif sensor1 >= distancetowall:
                turn_left()
                client.publish("state","l")
            else:
                turn_Around()
                client.publish("state","b")
                
    elif xx == 0:
        if facing == 90 or facing == -270: #right
            if sensor3 >= distancetowall:
                state=f"final/{xx}/{yy}/{facing}"
                client.publish("target",state)
                facingdum=facing
                tarx=xx
                tary=yy
                go="right"
                print(state,xx,yy,facing)
            if sensor2 >= distancetowall:
                forwardmap()
                client.publish("state","s")
            elif sensor1 >= distancetowall:
                turn_left()
                client.publish("state","l")
            else:
                turn_Around()
                client.publish("state","b")

        if abs(facing) == 180: #forward
            if sensor2 >= distancetowall:
                state=f"final/{xx}/{yy}/{facing}"
                client.publish("target",state)
                facingdum=facing
                tarx=xx
                tary=yy
                go = "forward"
                print(state,xx,yy,facing)
            if sensor3>= distancetowall:
                turn_right()
                client.publish("state","r")
            elif sensor1 >= distancetowall:
                turn_left()
                client.publish("state","l")
            else:
                turn_Around()
                client.publish("state","b")
        if facing == -90 or facing==270:
            if sensor1 >= distancetowall: #left
                state=f"final/{xx}/{yy}/{facing}"
                client.publish("target",state)
                facingdum=facing
                tarx=xx
                tary=yy
                go ="left"
                print(state,xx,yy,facing)
            if sensor3 >= distancetowall:
                turn_right()
                client.publish("state","r")
            elif sensor2 >= distancetowall:
                forwardmap()
                client.publish("state","s")
            else:
                turn_Around()
                client.publish("state","b")

    elif xx == 9:
        if facing == 0:
            if sensor2 >= distancetowall:
                state=f"final/{xx}/{yy}/{facing}"
                client.publish("target",state)
                facingdum=facing
                tarx=xx
                tary=yy
                go = "forward"
                print(state,xx,yy,facing)
            if sensor3 >= distancetowall:
                turn_right()
                client.publish("state","r")
            elif sensor1 >= distancetowall:
                turn_left()
                client.publish("state","l")
            else:
                turn_Around()
                client.publish("state","b")

        if facing == 90 or facing == -270:
            if sensor1 >= distancetowall: #left
                state=f"final/{xx}/{yy}/{facing}"
                client.publish("target",state)
                facingdum=facing
                tarx=xx
                tary=yy
                go = "left"
                print(state,xx,yy,facing)
            elif sensor3 >= distancetowall:
                turn_right()
                client.publish("state","r")
            if sensor2 >= distancetowall:
                forwardmap()
                client.publish("state","s")
            else:
                turn_Around()
                client.publish("state","b")
                
        if facing == -90 or facing == 270:
            if sensor3 >= distancetowall:
                state=f"final/{xx}/{yy}/{facing}"
                client.publish("target",state)
                facingdum=facing
                tarx=xx
                tary=yy
                go = "right"
                print(state,xx,yy,facing)
            if sensor2 >= distancetowall:
                forwardmap()
                client.publish("state","s")
            elif sensor1 >= distancetowall:
                turn_left()
                client.publish("state","l")
            else:
                turn_Around()
                client.publish("state","b")

    
running=True
while running:
    print("Grid",xx,yy)
    #print(f"Command = {latest_command}, Steps = {latest_steps}")
    # --- Read sensors ---``````
    sensor1 = sensor_raw1.distance*100  # left
    sensor2 = sensor_raw2.distance*100  # front
    sensor3 = sensor_raw3.distance*100  # right
    print(f"Left: {sensor1},Front: {sensor2},Right: {sensor3}")
    data = ser.readline().decode('utf-8', errors='ignore').strip()
    print(phase)
    print(f"Left: {sensor1:.2f},Front: {sensor2:.2f},Right: {sensor3:.2f}")
    if latest_dir == "start" :
        phase = 1
    if latest_command == "left":
        leftstep=latest_steps
        print(f"Now your left step is : {leftstep}")

    if latest_command == "right":
        rightstep=latest_steps
        print(f"Now your right step is : {rightstep}")

    if latest_command == "forward":
        forwardstep=latest_steps
        print(f"Now your forward step is : {forwardstep}")

    if latest_command == "around":
        aroundstep=latest_steps
        print(f"Now your around step is : {aroundstep}")

    if latest_command == "phase":
        phase=latest_steps
        print(f"Now your phase is : {phase}")

    if abs(facing) == 360:
        facing = 0
    
    if phase == 1 and a == 0:  # mapping phase
        # เลี้ยวขวา
        print(phase)
        check_exit()
        if sensor3 >= distancetowall:
            turn_right()
            client.publish("state","r")#right

        # กลับรถ
        elif sensor1 <= distancetowall and sensor2 <= distancetowall and sensor3 <= distancetowall:
            turn_Around()
            client.publish("state","b")#turn around

        # ตรงไป
        elif sensor2 >= distancetowall and sensor3 <= distancetowall:
            forwardmap()
            client.publish("state","s")#Forward

        # เลี้ยวซ้าย
        elif sensor1 >= distancetowall:
            turn_left()
            client.publish("state","l") #turn left

        if msgfromros == 'end':
            if facing == 90 or facing == -270:
                user_input = "left/" + str(leftstep)
                ser.write((user_input + "\n").encode('utf-8'))
                response = ""
                while response == "":
                    response = ser.readline().decode('utf-8').strip()
                time.sleep(0.5)

            elif abs(facing) == 180:
                user_input = "around/" + str(aroundstep)
                ser.write((user_input + "\n").encode('utf-8'))
                response = ""
                while response == "":
                    response = ser.readline().decode('utf-8').strip()
                time.sleep(0.5)

            elif facing == -90 or facing == 270:
                user_input = "right/" + str(rightstep)
                ser.write((user_input + "\n").encode('utf-8'))
                response = ""
                while response == "":
                    response = ser.readline().decode('utf-8').strip()
                time.sleep(0.5)
            a=1
            client.publish("state","stop")
            client.publish("state","solve")
            time.sleep(1)
            phase = 2

    elif phase == 2:  # solved phase
        if sensor1 >= distancetowall or sensor3 >= distancetowall:
            latest_dir='' 
            client.publish("state","ask")
            print("asked")
            time.sleep(2)
            if latest_dir == 'left':
                turn_left()
                
            elif latest_dir == 'right':
                turn_right()

            elif latest_dir == 'forward':
                forwardmap()
       
        else:
            user_input = "forward/" + str(forwardstep)
            ser.write((user_input + "\n").encode('utf-8'))
            response = ""
            while response == "":
                response = ser.readline().decode('utf-8').strip()
            time.sleep(0.5)
        if xx==tarx and yy == tary:
            if facing == facingdum:
                if go == "left":
                    turn_left()
                if go == "right":
                    turn_right()
                if go == "forward":
                    forwardmap()
            else:
                while facing != facingdum:
                    user_input = "right/" + str(leftstep)
                    ser.write((user_input + "\n").encode('utf-8'))
                    response =""
                    while response == "":
                        response = ser.readline().decode('utf-8').strip()
                    facing = (facing+90)%360

                if go == "left":
                    turn_left()
        
                if go == "right":
                   turn_right()

                if go == "forward":
                    forwardmap()

    

    prev_xx = xx
    prev_yy=yy
    if data:
        print(f"Received: {data},Left: {sensor1},Front: {sensor2},Right: {sensor3}")
    sensor1=0
    sensor2=0
    sensor3=0
client.loop_stop()
client.disconnect()
