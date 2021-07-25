#! /usr/bin/env python
import serial

import rospy
from std_msgs.msg import String

ser = serial.Serial("/dev/ttyUSB2", 9600)   #specify your port and braudrate
print "serial creado"

def callback(data):
    rospy.loginfo('Commando recibido: %s', data.data)
    ser.write(data.data + "\n")      # write a string
def listener():
    rospy.init_node('gripper_ros', anonymous=True)

    rospy.Subscriber('gripper_cmd', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def main():
  try:
    print "iniciando gripper control"
    ser.write("c 20\n")      # write a string

    # ser.close()             # close port
    listener()

  except rospy.ROSInterruptException:
    print "se mato el gripper"
    ser.close()             # close port
    return
  except KeyboardInterrupt:
    print "matanado el gripper"
    ser.close()             # close port
    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "matanado el gripper"
        ser.close()             # close port
        pass
