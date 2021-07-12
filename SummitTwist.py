import rospy
from geometry_msgs.msg import Twist
import time
from adafruit_servokit import ServoKit
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import Jetson.GPIO as GPIO

NEUTRAL = 90
MAX_STEER = 33

print("connecting to board")
i2c = busio.I2C(SCL, SDA)

pca = PCA9685(i2c)
print("connection complete")

pca.frequency = 50

print("setting up servos")


motor = servo.Servo(pca.channels[0])
steering0 = servo.Servo(pca.channels[1])
steering1 = servo.Servo(pca.channels[2])
shifter = servo.Servo(pca.channels[3])
lockRear = servo.Servo(pca.channels[4])
lockFront = servo.Servo(pca.channels[5])

print("setting throttle to neutral")
motor.angle = 90
print("throttle set to neutral")
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

def checkServoAngle(srv:servo):
    global NEUTRAL, MAX_STEER
    MIN_ANG = 0
    MAX_ANG = 180
    if srv == steering0 or srv == steering1:
        MIN_ANG = NEUTRAL - MAX_STEER
        MAX_ANG = NEUTRAL + MAX_STEER
    try:
        if srv.angle > MAX_ANG or srv.angle < MIN_ANG:
            srv.angle = 90
    except:
        print("servo angle was none")
        srv.angle = 90

def drive(speed:float):
    checkServoAngle(motor)
    if speed > 100:
        speed = 100
    elif speed < -100:
        speed = -100
    speed *= .3333
    speed += 90
    if motor.angle > speed:
        step = -1
    else:
        step = 1
    for i in range(int(motor.angle), int(speed), step):
        motor.angle = i
        time.sleep(0.05)

def steer(angle:float):
    global NEUTRAL, MAX_STEER
    checkServoAngle(steering0)
    checkServoAngle(steering1)
    angle *= -1
    toAngle = NEUTRAL + int(angle)
    if toAngle < NEUTRAL - MAX_STEER:
        toAngle = NEUTRAL - MAX_STEER
    elif toAngle > NEUTRAL + MAX_STEER:
        toAngle = NEUTRAL + MAX_STEER
    if int(steering0.angle) > int(toAngle):
        step = -1
    else:
        step = 1
    for i in range(int(steering0.angle), int(toAngle), step):
        steering0.angle = i
        steering1.angle = steering0.angle
        time.sleep(0.02)

def stopMotor():
    checkServoAngle(motor)
    if motor.angle is None:
        motor.angle = 90
    if (motor.angle < 90):
        step = 1
    else:
        step = -1
    for i in range(int(motor.angle), 90, step):
        motor.angle = i
        time.sleep(0.05)

def stopSteering():
    steer(0)
    sleepServo(steering0)
    sleepServo(steering1)

def stopAll():
    stopMotor()
    if steering0.angle is not None or steering1.angle is not None:
        stopSteering()
    if shifter.angle is not None:
        shiftLow()
    if lockFront.angle is not None:
        unlockF()
    if lockRear.angle is not None:
        unlockR()

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

print("definitions complete")

class Summit:
    def __init__(self):
        rospy.init_node('summit')

        self._last_received = rospy.get_time()
        self._timeout = 1.0
        self._rate = 10
        self._max_speed = 0.5
        self._wheel_base = 0.3
        
        self._speed_percent = 0.0
        self._steering_percent = NEUTRAL

        rospy.Subscriber('cmd_vel', Twist, self.velocity_received_callback)

    def velocity_received_callback(self, message):
        self._last_received = rospy.get_time()

        linear = message.linear.x

        if linear > 1.0:
            linear = 1.0
        elif linear < -1.0:
            linear = -1.0

        angular = message.angular.z

        if angular > 1.0:
            angular = 1.0
        elif angular < -1.0:
            angular = -1.0

        self._speed_percent = linear * 100.0
        self._steering_percent = angular * 100.0

        print("//----------------------//")
        print("COMMAND:")
        print("in angular: " + str(angular))
        print("in linear: " + str(linear))
        print("got speed!")
        print("out speed: " + str(round(self._speed_percent,1)) + "%")
        print("out turn: " + str(round(self._steering_percent,1)) + "%")

    def run(self):
        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            delay = rospy.get_time() - self._last_received

            if delay < self._timeout:
                drive(self._speed_percent)
                steer(self._steering_percent)
            else:
                stopAll()
            
            rate.sleep()

def main():
    summit = Summit()

    summit.run()

if __name__ == '__main__':
    main()


