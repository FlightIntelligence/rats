#!/usr/bin/env python

import rospy
import requests
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from ros_vive_driver_msgs.msg import ControllerState


box_dimensions = [10, 10, 5]  # given in rats coordinates, x, y, z
ugo_server = "http://192.168.0.112:9000/location/"

_box_dimensions = [box_dimensions[0]/2.0, box_dimensions[2]/2.0, box_dimensions[2]/2.0]
_previous_position_code = ''
_learning = True

def _callback_controller_state(data):
    global _learning

    if data.trigger <= 0.001:
        _learning = False
    else:
        _learning = True

    print(_learning)

def _callback_controller_position(data):
    global _previous_position_code

    if _learning == False:
        return

    # print("Received data" + str(data))

    position_code = _get_position_code(data.pose.position.x, data.pose.position.y, data.pose.position.z, "L")

    try:
        if position_code != _previous_position_code:
            response = requests.get(ugo_server + position_code)
            _previous_position_code = position_code
            print(position_code)
    except requests.ConnectionError:
        rospy.loginfo("Could not connect to Ugo")
        pass



def _callback_tracker_position(data):
    """ treat the data
    # convert the data into proper alphabet letter
    # send to Ugo computer
    """
    global _previous_position_code

    if _learning == True:
        return

    print("TRACKER: Received data" + str(data))

    position_code = _get_position_code(data.pose.position.x, data.pose.position.y, data.pose.position.z, "F")

    try:
        if position_code != _previous_position_code:
            response = requests.get(ugo_server + position_code)
            _previous_position_code = position_code
            print(position_code)
    except requests.ConnectionError:
        rospy.loginfo("Could not connect to Ugo")
        pass


def _get_position_code(current_x, current_y, current_z, code):
    """
        Converts the current coordinate from a drone to a letter code, according to Ugo drawing.

        The conversion is basically spliting a cube volume into 8 smaller cubes, each one having a different
        letter code.

    :param current_x: position x
    :param current_y: position y
    :param current_z: position z (high)

    :return: letter code
    """

    letter_code = code

    if current_y <= 0.0:
        letter_code += 'B'
    else:
        letter_code += 'F'

    if current_x <= 0:
        letter_code += 'L'
    else:
        letter_code += 'R'

    if current_z <= 1.3:
        letter_code += 'D'
    else:
        letter_code += 'U'

    if current_z <= 0.01:
        letter_code = 'END'

    return letter_code



def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/vive/controller_0/state", ControllerState, _callback_controller_state)
    rospy.Subscriber("/vive/world_tracker_0", PoseStamped, _callback_tracker_position)
    rospy.Subscriber("/vive/world_controller_0", PoseStamped, _callback_controller_position)

    print("Loaded broadcaster")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass