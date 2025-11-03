import serial
import time
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import DistanceSensor
from time import sleep
import paho.mqtt.client as mqtt

BROKER = "test.mosquitto.org"   # public test broker (no auth). Replace with your broker.
PORT = 1883
TOPIC = "step"

try:
    ser = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=0.5)
    time.sleep(2)  # Give time for UART to initialize
    print("✅ Serial port open on /dev/ttyAMA0 (9600 baud)")
except Exception as e:
    print(f"❌ Failed to open serial port: {e}")
    sys.exit(1)

leftstep = 800
rightstep = 800
aroundstep = 800
forwardstep = 800
a=0
# --- Current position tracking ---
xx, yy = 0, 0  # current position
prev_xx, prev_yy = 0, 0  # previous position
facing = 0
ultra1 = 0
ultra2 = 0
distancetowall = 10
phase = 0  # 1 = mapping, 2 = solved, 3 = return
msgtoros = ''
msgfromros = ''
sensor_raw1 = DistanceSensor(echo=27, trigger=17)  # left
sensor_raw2 = DistanceSensor(echo=24, trigger=23)  # front
sensor_raw3 = DistanceSensor(echo=6, trigger=5)    # right
# --- Main Loop ---
running = True

def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe("step")

latest_command=''
latest_steps=0

def on_message(client, userdata, msg):
    global latest_command, latest_steps
    payload = msg.payload.decode()
    print(f"Received: {payload}")

    # ✅ Split "command/number" format
    if "/" in payload:
        command, steps_str = payload.split("/", 1)  # split only at first "/"
        latest_command = command
        try:
            latest_steps = int(steps_str)
        except ValueError:
            print(f"Invalid number: {steps_str}")
            latest_steps = None
    else:
        print("Invalid format, expected command/number")


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("test.mosquitto.org", 1883, 60)

# ✅ Start MQTT network loop in background
client.loop_start()

def forwardmap(xx,yy,prev_xx,prev_yy):
    user_input = "forwardmap/" + str(forwardstep)
    ser.write((user_input + "\n").encode('utf-8'))
    response = ""
    while response != "forwardmap":
        #xx= 
        #yy=
        if xx != prev_xx or yy != prev_yy:
            user_input = "stop/" + str(0)
            ser.write((user_input + "\n").encode('utf-8'))
        response = ser.readline().decode('utf-8').strip()
        prev_xx = xx
        prev_yy=yy

while running:
    print(f"Command = {latest_command}, Steps = {latest_steps}")
    # --- Read sensors ---``````
    sensor1 = sensor_raw1.distance*100  # left
    sensor2 = sensor_raw2.distance*100  # front
    sensor3 = sensor_raw3.distance*100  # right
    print(phase)
    if latest_command == "start" and latest_steps == 1:
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

    print(f"Left: {sensor1:.2f},Front: {sensor2:.2f},Right: {sensor3:.2f}")
    if phase == 1 and a == 0:  # mapping phase
        # เลี้ยวขวา
        print(phase)
        if sensor3 >= distancetowall:
            user_input = "right/" + str(rightstep)
            ser.write((user_input + "\n").encode('utf-8'))
            print("right")

            response = ""
            while response == "":
                response = ser.readline().decode('utf-8').strip()
            time.sleep(0.5)

            user_input = "forward/" + str(forwardstep)
            ser.write((user_input + "\n").encode('utf-8'))
            response = ""
            while response == "":
                response = ser.readline().decode('utf-8').strip()
            facing = facing + 90
            time.sleep(0.5)

        # กลับรถ
        elif sensor1 <= distancetowall and sensor2 <= distancetowall and sensor3 <= distancetowall:
            user_input = "around/" + str(aroundstep)
            ser.write((user_input + "\n").encode('utf-8'))
            print("around")

            response = ""
            while response == "":
                response = ser.readline().decode('utf-8').strip()
            time.sleep(0.5)

            user_input = "forward/" + str(forwardstep)
            ser.write((user_input + "\n").encode('utf-8'))
            response = ""
            while response == "":
                response = ser.readline().decode('utf-8').strip()
            facing = facing + 180
            time.sleep(0.5)

        # ตรงไป
        elif sensor2 >= distancetowall and sensor3 <= distancetowall:
            user_input = "forward/" + str(forwardstep)
            ser.write((user_input + "\n").encode('utf-8'))
            print("forward")

            response = ""
            while response == "":
                response = ser.readline().decode('utf-8').strip()
            time.sleep(0.5)

        # เลี้ยวซ้าย
        elif sensor1 >= distancetowall:
            user_input = "left/" + str(leftstep)
            ser.write((user_input + "\n").encode('utf-8'))
            print("left")

            response = ""
            while response == "":
                response = ser.readline().decode('utf-8').strip()
            time.sleep(0.5)

            user_input = "forward/" + str(forwardstep)
            ser.write((user_input + "\n").encode('utf-8'))
            response = ""
            while response == "":
                response = ser.readline().decode('utf-8').strip()
            facing = facing - 90
            time.sleep(0.5)




        if msgtoros == 'end':
            if facing == 90:
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
            time.sleep(1)
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            if data:
                if msgfromros == "round2":
                    phase = 2

    elif phase == 2:  # solved phase
        if sensor1 >= distancetowall or sensor3 >= distancetowall:
            msgtoros = 'ask'

            if msgfromros == 'left':
                user_input = "left/" + str(leftstep)
                ser.write((user_input + "\n").encode('utf-8'))
                response =""
                while response == "":
                    response = ser.readline().decode('utf-8').strip()

                user_input = "forward/" + str(forwardstep)
                ser.write((user_input + "\n").encode('utf-8'))
                response = ""
                while response == "":
                    response = ser.readline().decode('utf-8').strip()
                time.sleep(0.5)

            elif msgfromros == 'right':
                user_input = "right/" + str(rightstep)
                ser.write((user_input + "\n").encode('utf-8'))
                response=""
                while response == "":
                    response = ser.readline().decode('utf-8').strip()

                user_input = "forward/" + str(forwardstep)
                ser.write((user_input + "\n").encode('utf-8'))
                response = ""
                while response == "":
                    response = ser.readline().decode('utf-8').strip()
                time.sleep(0.5)

            elif msgfromros == 'forward':
                user_input = "forward/" + str(forwardstep)
                ser.write((user_input + "\n").encode('utf-8'))
                response=""
                while response == "":
                    response = ser.readline().decode('utf-8').strip()

                time.sleep(0.5)
       
        else:
            user_input = "forward/" + str(forwardstep)
            ser.write((user_input + "\n").encode('utf-8'))
            response = ""
            while response == "":
                response = ser.readline().decode('utf-8').strip()
            time.sleep(0.5)

        if msgfromros == 'end':
            user_input = "around/" + str(aroundstep)
            ser.write((user_input + "\n").encode('utf-8'))
            response = ""
            while response == "":
                response = ser.readline().decode('utf-8').strip()
            time.sleep(0.5)
            phase = 3

    elif phase == 3:  # return phase
        if sensor1 >= distancetowall or sensor3 >= distancetowall:
            msgtoros = 'ask2'

            if msgfromros == 'left':
                user_input = "left/" + str(leftstep)
                ser.write((user_input + "\n").encode('utf-8'))
                response = ""
                while response == "":
                    response = ser.readline().decode('utf-8').strip()

                user_input = "forward/" + str(forwardstep)
                ser.write((user_input + "\n").encode('utf-8'))
                response = ""
                while response == "":
                    response = ser.readline().decode('utf-8').strip()
                time.sleep(0.5)

            elif msgfromros == 'right':
                user_input = "right/" + str(rightstep)
                ser.write((user_input + "\n").encode('utf-8'))
                response = ""
                while response == "":
                    response = ser.readline().decode('utf-8').strip()

                user_input = "forward/" + str(forwardstep)
                ser.write((user_input + "\n").encode('utf-8'))
                response = ""
                while response == "":
                    response = ser.readline().decode('utf-8').strip()
                time.sleep(0.5)

            elif msgfromros == 'forward':
                user_input = "forward/" + str(forwardstep)
                ser.write((user_input + "\n").encode('utf-8'))
                response = ""
                while response == "":
                    response = ser.readline().decode('utf-8').strip()
                time.sleep(0.5)
        else:
            user_input = "forward/" + str(forwardstep)
            ser.write((user_input + "\n").encode('utf-8'))
            response = ""
            while response == "":
                response = ser.readline().decode('utf-8').strip()
            time.sleep(0.5)

        if msgfromros == 'end':
            user_input = "around/" + str(aroundstep)
            ser.write((user_input + "\n").encode('utf-8'))
            response = ""
            while response == "":
                response = ser.readline().decode('utf-8').strip()
            time.sleep(0.5)
            phase = 3

    if abs(facing) == 360:
        facing = 0

    # --- Read incoming UART data ---
    data = ser.readline().decode('utf-8', errors='ignore').strip()

    if data:
        print(f"Received: {data},Left: {sensor1},Front: {sensor2},Right: {sensor3}")
    
client.loop_stop()
client.disconnect()