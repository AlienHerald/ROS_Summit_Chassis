# ROS_Summit_Chassis
Repo containing services scripts and other miscellaneous code for the Summit Autonomous Chassis

## Services
BatteryMonitor_Node_Summit and SummitTwist are meant to be run as services.

## Python Script
SummitCamera contains a text entry field for moving camera servoes and controlling servos on the chassis like the front/rear diffs and shifter and lights.

## Arduino Code
Adapted from https://raspi.tv/2015/raspio-duino-as-a-lipo-monitor this tutorial has a great example of how to wire up the resistors to monitor the voltages out of the cells of lipo battery. Just remember that the bigger resistor comes first in the circuit once you start getting to 3S and 4S and larger batteries!

### TODO
Create a websockets html page that contains the functionality of SummitCamera and turn the script into a ROS node that can run on start.
