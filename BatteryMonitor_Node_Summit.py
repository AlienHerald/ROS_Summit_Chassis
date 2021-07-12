# For ROS node
import rospy
from std_msgs.msg import String

# For Arduino serial read
import serial
import subprocess
import sys
from time import sleep

subprocess.Popen('clear', shell=True)

bad_threshold = 0.15


def calculate_4s_percentage(cell0, cell1, cell2, cell3):
    if (abs(cell0 - cell1) > bad_threshold and abs(cell0 - cell2) > bad_threshold and abs(cell0 - cell3) > bad_threshold):
        cell0 = cell1

    if (abs(cell1 - cell0) > bad_threshold and abs(cell1 - cell2) > bad_threshold and abs(cell1 - cell3) > bad_threshold):
        cell1 = cell0

    if (abs(cell2 - cell0) > bad_threshold and abs(cell2 - cell1) > bad_threshold and abs(cell2 - cell3) > bad_threshold):
        cell2 = cell0
        
    battery_percentage = (cell0 + cell1 + cell2 + cell3) / 4.0
    battery_percentage -= 3.2
    battery_percentage *= 100.0
    return int(battery_percentage)

def calculate_2s_percentage(cell0, cell1):
    battery_percentage = (cell0 + cell1) / 2.0
    battery_percentage -= 3.2
    battery_percentage *= 100.0
    return int(battery_percentage)

def battery_status():
    pub = rospy.Publisher('Summit_Battery', String, queue_size=10)
    rospy.init_node('Summit_Battery')
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        battery_string = 'undefined'
        for i in range(0, 3, 1):
            port = '/dev/ttyACM' + str(i)
            status_string = 'Trying port ' + port
            rospy.loginfo(status_string)
            sleep(1)
            try:
                sport = serial.Serial(port, 9600, timeout=3)
                rospy.loginfo('Connected to ' + port)
                response = sport.readline()
                sport.close()
                rospy.loginfo('Read line')

                decoded = response.decode()
                stripped_string = decoded.strip()
                values_array = stripped_string.split(',')
                rospy.loginfo('Decoded and split, ' + str(values_array))

                battery_main0 = calculate_4s_percentage(float(values_array[0]), float(values_array[1]), float(values_array[2]), float(values_array[3]))
                battery_main1 = calculate_4s_percentage(float(values_array[5]), float(values_array[6]), float(values_array[7]), float(values_array[8]))

                battery_main = float(battery_main0 + battery_main1) / 2.0
                battery_main = int(battery_main)
                if (battery_main < 0):
                    battery_main = 'disconnected'
                rospy.loginfo('Calculated main battery, ' + str(battery_main))

                battery0 = calculate_2s_percentage(float(values_array[9]), float(values_array[10]))
                battery1 = calculate_2s_percentage(float(values_array[9]), float(values_array[10]))

                battery_drive = float(battery0 + battery1) / 2.0
                battery_drive = int(battery_drive)
                if (battery_drive < 0):
                    battery_drive = 'disconnected'
                rospy.loginfo('Calculated drive battery, ' + str(battery_drive))

                battery_string = str(battery_main) + ',' + str(battery_drive)

                rospy.loginfo('Exiting loop')
                sleep(10)
                flag = True
            except:
                rospy.loginfo('Failed to connect to ' + port)
                continue

        rospy.loginfo(battery_string)
        pub.publish(battery_string)
        rate.sleep()

if __name__ == '__main__':
    try:
        battery_status()
    except rospy.ROSInterruptException:
        pass
