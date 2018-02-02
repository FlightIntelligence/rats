import os

import flask
import sys
from flask import json, request
from SwarmBootstrapUtils import executor
import demo_backend
import subprocess

my_env = os.environ.copy()
my_env['ROS_MASTER_URI'] = 'http://127.0.0.1:11311'

tracker = dict()
tracker['processes'] = list()
tracker['opened_files'] = list()

logging_path = "/tmp/log"


launcher = demo_backend.DemoLauncher()

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

    try:
        executor.send_new_mission(my_env=my_env, tracker=tracker, log_dir=logging_path, mission=ros_topic_message)

        response = flask.Response(remote_response_content, status=200)
    except ValueError as err:
        return flask.Response(str(err), status=409)

    return response


@child_server.route('/launch', methods=['POST'])
def launch():
    data_object = flask.request.get_json()

    if 'config_dir' in data_object:
        config_dir = data_object['config_dir']
    else:
        return flask.Response('config_dir not found', status=400)

    if 'drone_ips' in data_object:
        drone_ips = data_object['drone_ips']
    else:
        return flask.Response('drone_ips not found', status=400)

    try:
        launcher.launch(config_dir, drone_ips)
    except ValueError as err:
        return flask.Response(str(err), status=400)

    return flask.Response(status=202)


@child_server.route('/stop', methods=['POST'])
def stop():
    try:
        common_land()
        launcher.stop()
    except ValueError as err:
        return flask.Response(str(err), status=409)

    return flask.Response(status=202)


@child_server.route('/status', methods=['GET'])
def get_status():
    parent_status = 'NO_INFORMATION'
    status = launcher.get_status()

    if status == demo_backend.DemoLauncher.Status.IDLE:
        parent_status = "IDLE"
    elif status == demo_backend.DemoLauncher.Status.LAUNCHING:
        parent_status = 'LAUNCHING'
    elif status == demo_backend.DemoLauncher.Status.READY:
        parent_status = 'READY'
    elif status == demo_backend.DemoLauncher.Status.FLYING:
        parent_status = 'FLYING'
    elif status == demo_backend.DemoLauncher.Status.STOPPING:
        parent_status = 'STOPPING'

    return flask.Response(parent_status, status=200)


@child_server.route('/land', methods=['POST', 'GET'])
def common_land():
    executor.land(my_env=my_env, tracker=tracker, log_dir=logging_path)
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
