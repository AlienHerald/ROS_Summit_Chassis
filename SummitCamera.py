#!usr/bin/python3

import time
from adafruit_motor import servo
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import Jetson.GPIO as GPIO

MOVING = False

CAM0_NEUTRAL = 135
CAM1_NEUTRAL = 130

CAM0_MIN = 15
CAM0_MAX = 255

CAM1_MIN = 100
CAM1_MAX = 170

print("connecting to board")
i2c = busio.I2C(SCL, SDA)

pca = PCA9685(i2c)
print("connection complete")

pca.frequency = 50

print("setting up servos")

shifter = servo.Servo(pca.channels[3])
lockRear = servo.Servo(pca.channels[4])
lockFront = servo.Servo(pca.channels[5])

cam0 = servo.Servo(pca.channels[6])
cam0.actuation_range = 270
cam1 = servo.Servo(pca.channels[7])
cam1.actuation_range = 270
lightsA = servo.Servo(pca.channels[8])
lightsB = servo.Servo(pca.channels[9])

print("enabling relay pin")
ESCPIN = 33
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ESCPIN, GPIO.OUT, initial=GPIO.LOW)
print("relay pin enabled, esc ready to activate")

print("making definitions")
def enableESC():
    GPIO.output(ESCPIN, GPIO.HIGH)

def disableESC():
    GPIO.output(ESCPIN, GPIO.LOW)

def lightsOn():
    lightsA.angle = 150
    lightsB.angle = 150

def lightsOff():
    lightsA.angle = 50
    lightsB.angle = 50

def checkServoAngle(srv:servo):
    MIN_ANG = 0
    if srv == cam0:
        MAX_ANG = 270
        NEUTRAL = 135
    else:
        MAX_ANG = 180
        NEUTRAL = 90
    try:
        if srv.angle > MAX_ANG or srv.angle < MIN_ANG:
            srv.angle = NEUTRAL
    except:
        print("angle was none")
        srv.angle = NEUTRAL

print("checking servo angles")
checkServoAngle(cam0)
checkServoAngle(cam1)

def moveCamServo(srv:servo, ang:float):
    checkServoAngle(srv)
    MOVING = True
    if srv == cam0:
        if ang > CAM0_MAX:
            ang = CAM0_MAX
        elif ang < CAM0_MIN:
            ang = CAM0_MIN
    elif srv == cam1:
        if ang > CAM1_MAX:
            ang = CAM1_MAX
        elif ang < CAM1_MIN:
            ang = CAM1_MIN
    if srv.angle > ang:
        step = -1
    else:
        step = 1
    for i in range(int(srv.angle), int(ang), step):
        srv.angle = i
        time.sleep(0.05)
    MOVING = False

def stopCam():
    moveCamServo(cam0, CAM0_NEUTRAL)
    moveCamServo(cam1, CAM1_NEUTRAL)

def scan():
    moveCamServo(cam0, CAM0_MIN)
    moveCamServo(cam0, CAM0_MAX)
    moveCamServo(cam0, CAM0_NEUTRAL)

def sleepServo(srv:servo):
    time.sleep(0.05)
    srv.angle = None

def shiftHigh():
    shifter.angle = 155
    sleepServo(shifter)

def shiftLow():
    shifter.angle = 0
    sleepServo(shifter)

def lockF():
    lockFront.angle = 145
    sleepServo(lockFront)

def unlockF():
    lockFront.angle = 15
    sleepServo(lockFront)

def lockR():
    lockRear.angle = 145
    sleepServo(lockRear)

def unlockR():
    lockRear.angle = 15
    sleepServo(lockRear)

print("Camera servos active. Please wait.")

while True:
    move = input("Enter move type: ")
    if move == ']':
        print("tilt min")
        moveCamServo(cam0, CAM0_MIN)
    elif move == '[':
        print("tilt max")
        moveCamServo(cam0, CAM0_MAX)
    elif move == '=':
        print("scan min")
        moveCamServo(cam1, CAM1_MIN)
    elif move == '-':
        print("scan max")
        moveCamServo(cam1, CAM1_MAX)
    elif move == ';':
        moveCamServo(cam1, 140)
    elif move == 'start':
        print('setting to neutral for start')
        moveCamServo(cam0, 90)
        moveCamServo(cam1, CAM1_NEUTRAL)
    elif move == 'scan':
        print("scanning")
        moveCamServo(cam0, CAM0_MIN)
        print("scanning...")
        moveCamServo(cam0, CAM0_MAX)
        print("...scanning")
        moveCamServo(cam0, CAM0_NEUTRAL)
        print("scan complete")
    elif move == 'on':
        print("lights on")
        lightsOn()
    elif move == 'off':
        print("lights off")
        lightsOff()
    elif move == 'down':
        print("look down")
        moveCamServo(cam1, 155)
    elif move == 'lockf':
        print("lock front")
        lockF()
    elif move == 'unlockf':
        print("unlock front")
        unlockF()
    elif move == 'lockr':
        print("lock rear")
        lockR()
    elif move == 'unlockr':
        print("unlock rear")
        unlockR()
    elif move == 'shifthigh':
        print("shift high")
        shiftHigh()
    elif move == 'shiftlow':
        print("shiftlow")
        shiftLow()
    elif move == 'escon':
        print("activating esc")
        enableESC()
    elif move == 'escoff':
        print("disabling esc")
        disableESC()
    elif MOVING == False:
        print("reset")
        stopCam()
