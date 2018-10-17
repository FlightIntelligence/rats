#!/usr/bin/env python

import rospy
import requests
from std_msgs.msg import String
from geometry_msgs.msg import Pose


box_dimensions = [10, 10, 5]  # given in rats coordinates, x, y, z
ugo_server = "http://192.168.0.112:9000/location/"

_box_dimensions = [box_dimensions[0]/2.0, box_dimensions[2]/2.0, box_dimensions[2]/2.0]
_previous_position_code = ''

def _callback(data):
    """ treat the data
    # convert the data into proper alphabet letter
    # send to Ugo computer
    """
    global _previous_position_code
    print("Received data" + str(data))
    
    position_code = _get_position_code(data.position.x, data.position.y, data.position.z)
    print (position_code)
    try:
        if position_code != _previous_position_code:
            response = requests.get(ugo_server + position_code)
            _previous_position_code = position_code
    except requests.ConnectionError:
        rospy.loginfo("Could not connect to Ugo")
        pass


def _get_position_code(current_x, current_y, current_z):
    """
        Converts the current coordinate from a drone to a letter code, according to Ugo drawing.

        The conversion is basically spliting a cube volume into 8 smaller cubes, each one having a different
        letter code.

    :param current_x: position x
    :param current_y: position y
    :param current_z: position z (high)

    :return: letter code
    """

    letter_code = 'F'
    if current_x <= _box_dimensions[0]:
        letter_code += 'B'
    else:
        letter_code += 'F'

    if current_y <= _box_dimensions[1]:
        letter_code += 'L'
    else:
        letter_code += 'R'

    if current_z <= _box_dimensions[2]:
        letter_code += 'D'
    else:
        letter_code += 'U'

    return letter_code



def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("bebop_drone_pose", Pose, _callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass