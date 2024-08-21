#!/usr/bin/env python3

import serial
import time
import math

# Initialize the serial port
port = "/dev/ttyUSB0"  # define particular port used
ser = serial.Serial(port, 1000000)  # define serial port
ser.close()  # close port if previously open
ser.open()  # open port
print(ser.isOpen())  # make sure we could open the port!

# Define functions to calculate the Goal Position
def GoalPositionValH(angle):
    if math.floor(angle * 3.41) > 256:
        return int(math.floor(angle * 3.41 / 256))
    else:
        return 0

def GoalPositionValL(angle):
    if math.floor(angle * 3.41) < 256:
        return int(math.floor(angle * 3.41))
    else:
        more = int(math.floor(angle * 3.41 / 256))
        return int(math.floor(angle * 3.41 - more * 256))

def checksum(id, length, Write_instruction, GoalPositionAddress, DegL, DegH):
    CHECKSUM = id + length + Write_instruction + GoalPositionAddress + DegL + DegH
    more = int(math.floor(CHECKSUM / 256))
    return 255 - int(math.floor(CHECKSUM - more * 256))

start = [255, 255]
length = 5
Write_instruction = 3
GoalPositionAddress = 30

# Function to write the position to a servo
def write_position(id, angle):
    DegH = GoalPositionValH(angle)
    DegL = GoalPositionValL(angle)
    CHECKSUM = checksum(id, length, Write_instruction, GoalPositionAddress, DegL, DegH)
    print(f"ID {id}: Moving to {angle} degrees with checksum = {CHECKSUM}")
    return "".join(map(chr, start + [id, length, Write_instruction, GoalPositionAddress, DegL, DegH, CHECKSUM]))

# Function to move the servo to the specified angle
def move_servo(id, angle):
    ser.write(write_position(id, angle).encode('latin-1'))
    time.sleep(0.5)  # Increase the delay for slower movement

# Function to control the gripper with angle constraint
def control_gripper(id, angle):
    if 140 <= angle <= 180:
        move_servo(id, angle)
    else:
        print(f"Gripper angle {angle} is out of range. Must be between 140 and 180 degrees.")

try:
    # Initialize positions
    control_gripper(1, 180)  # Initialize gripper to 180 degrees
    move_servo(6, 40)  # Initialize joint to 40 degrees
    move_servo(3, 260)  # Initialize base to 260 degrees

    # Move gripper from 180 to 140 degrees
    control_gripper(1, 140)
    # Move gripper back to 180 degrees
    #control_gripper(1, 180)

    # Move joint (ID 6) to 200 degrees, then to 90 degrees
    move_servo(6, 200)
    move_servo(6, 90)

    # Move base (ID 3) from 260 to 210 degrees
    move_servo(3, 210)

    # Return to initial positions
    control_gripper(1, 180)  # Gripper back to 180 degrees
    move_servo(6, 40)  # Joint back to 40 degrees
    move_servo(3, 260)  # Base back to 260 degrees

except KeyboardInterrupt:
    print("Stopping the servo control")

finally:
    ser.close()  # Close the serial port when done

