import os

import flask
import sys
from flask import json, request
import backend
import subprocess
from SwarmBootstrapUtils import executor

launcher = backend.Launcher()

child_server = flask.Flask(__name__)

@child_server.route('/', methods=['POST', 'GET'])
def main():
    return flask.render_template('demo_main_control.html')

@child_server.route('/new_mission', methods=['POST'])
def new_mission():
    response = ''
    remote_response_content = request.get_json()
    x = remote_response_content['x_coordinates']
    y = remote_response_content['y_coordinates']

    list_waypoints = list()
    for i in range(0, 600):
        list_waypoints.append({'position': [x[i], y[i], 0]})

    ros_topic_message = {'header': 'auto', 'poses': list_waypoints}

    my_env = os.environ.copy()
    my_env['ROS_MASTER_URI'] = 'http://127.0.0.1:11311'

    tracker = dict()
    tracker['processes'] = list()
    tracker['opened_files'] = list()

    try:
        executor.send_new_mission(my_env=my_env, tracker=tracker, log_dir="/tmp/log", mission=ros_topic_message)

        response = flask.Response(remote_response_content, status=200)
    except ValueError as err:
        return flask.Response(str(err), status=409)

    return response


@child_server.route('/launch', methods=['POST'])
def launch():
    data = flask.request.get_json()
    data_object = json.loads(data)
    if 'config_dir' in data_object:
        config_dir = data_object['config_dir']
    else:
        return flask.Response('config_dir not found', status=400)

    if 'drone_ips' in data_object:
        drone_ips = data_object['drone_ips']
    else:
        return flask.Response('drone_ips not found', status=400)

    launcher.launch(config_dir, drone_ips)
    return flask.Response(status=202)


@child_server.route('/start-flying', methods=['POST'])
def start_flying():
    try:
        launcher.start_flying()
    except ValueError as err:
        return flask.Response(str(err), status=409)

    return flask.Response(status=202)


@child_server.route('/stop', methods=['POST'])
def stop():
    try:
        launcher.stop()
    except ValueError as err:
        return flask.Response(str(err), status=409)

    return flask.Response(status=202)


@child_server.route('/status', methods=['GET'])
def get_status():
    return str(launcher.get_status())


@child_server.route('/takeoff', methods=['POST', 'GET'])
def common_takeoff():
    my_env = os.environ.copy()
    my_env['ROS_MASTER_URI'] = 'http://localhost:11311'
    cmd = 'rostopic pub -1 /common/takeoff std_msgs/Empty'
    subprocess.Popen(cmd.split(), env=my_env)
    return flask.Response(status=202)


@child_server.route('/land', methods=['POST', 'GET'])
def common_land():
    my_env = os.environ.copy()
    my_env['ROS_MASTER_URI'] = 'http://localhost:11311'
    cmd = 'rostopic pub -1 /common/land std_msgs/Empty'
    subprocess.Popen(cmd.split(), env=my_env)
    return flask.Response(status=202)


@child_server.route('/restart', methods=['POST', 'GET'])
def restart():
    command = "/sbin/reboot"
    subprocess.call(command, shell=True)
    return flask.Response(status=202)


@child_server.route('/shutdown', methods=['POST', 'GET'])
def shutdown():
    command = "/sbin/shutdown -h now"
    subprocess.call(command, shell=True)
    return flask.Response(status=202)


if __name__ == '__main__':
    host_ip = str(sys.argv[1])
    port = int(sys.argv[2])
    child_server.run(host=host_ip, port=port, threaded=True)

