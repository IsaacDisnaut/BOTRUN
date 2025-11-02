import serial
import time
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import DistanceSensor
from time import sleep

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

# --- Current position tracking ---
xx, yy = 0, 0  # current position
prev_xx, prev_yy = 0, 0  # previous position
facing = 0
ultra1 = 0
ultra2 = 0
distancetowall = 10
phase = 1  # 1 = mapping, 2 = solved, 3 = return
msgtoros = ''
msgfromros = ''
sensor_raw1 = DistanceSensor(echo=24, trigger=23)  # left
sensor_raw2 = DistanceSensor(echo=27, trigger=17)  # front
sensor_raw3 = DistanceSensor(echo=6, trigger=5)    # right
# --- Main Loop ---
running = True
while running:
    # --- Read sensors ---
    sensor1 = sensor_raw1.distance*100  # left
    sensor2 = sensor_raw2.distance*100  # front
    sensor3 = 0  # right

    print(f"Left: {sensor1:.2f},Front: {sensor2:.2f},Right: {sensor3:.2f}")
    if phase == 1:  # mapping phase
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
                while response == "":
                    response = ser.readline().decode('utf-8').strip()
                time.sleep(0.5)

            elif abs(facing) == 180:
                user_input = "around/" + str(aroundstep)
                ser.write((user_input + "\n").encode('utf-8'))
                while response == "":
                    response = ser.readline().decode('utf-8').strip()
                time.sleep(0.5)

            elif facing == -90 or facing == 270:
                user_input = "right/" + str(rightstep)
                ser.write((user_input + "\n").encode('utf-8'))
                while response == "":
                    response = ser.readline().decode('utf-8').strip()
                time.sleep(0.5)

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
                while response == "":
                    response = ser.readline().decode('utf-8').strip()
                time.sleep(0.5)

            elif msgfromros == 'right':
                user_input = "right/" + str(rightstep)
                ser.write((user_input + "\n").encode('utf-8'))
                while response == "":
                    response = ser.readline().decode('utf-8').strip()
                time.sleep(0.5)

            elif msgfromros == 'forward':
                user_input = "forward/" + str(forwardstep)
                ser.write((user_input + "\n").encode('utf-8'))
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
                time.sleep(0.5)

            elif msgfromros == 'right':
                user_input = "right/" + str(rightstep)
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
