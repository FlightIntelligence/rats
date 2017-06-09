import flask
import sys
from flask import json, render_template, url_for, request
import requests
import Pinger
import gevent
from SwarmBootstrapUtils import executor

child_server_ips = [('192.168.13.108', 8080)]

# config_dir = 0
# config_dir = 'rats_show/full_show'

parent_server = flask.Flask(__name__)


@parent_server.route('/', methods=['POST', 'GET'])
def main():
    return flask.render_template('demo_main_control.html')

@parent_server.route('/new_mission', methods=['POST'])
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
        executor.send_new_mission(my_env="", tracker="", log_dir="", mission=ros_topic_message)

        response = flask.Response(remote_response_content, status=200)
    except ValueError as err:
        return flask.Response(str(err), status=409)

    return response


@parent_server.route('/start_flying', methods=['GET'])
def start_flying():
    global deadmanswitch_state
    deadmanswitch_state = '0'
    
    response = ''
    remote_response_content = ''
    
    try:
        start_flying_ok = True
        for conf in child_server_ips:
            response = send_start_flying(conf[0], conf[1])
            remote_response_content += str(response.content)
            
            if not response.status_code == 202:
                start_flying_ok = False
            
        
        if start_flying_ok:
            response = flask.Response("Starting the Show", status=202)
        else:
            response = flask.Response(remote_response_content, status=409)
            
    except ValueError as err:
        return flask.Response(str(err), status=409)

    return response

@parent_server.route('/takeoff', methods=['GET'])
def takeoff():
    response = ''
    remote_response_content = ''
    
    try:
        start_taking_ok = True
        for conf in child_server_ips:
            response = send_takeoff(conf[0], conf[1])
            remote_response_content += str(response.content)
            
            if not response.status_code == 202:
                start_taking_ok = False
            
        
        if start_taking_ok:
            response = flask.Response("Taking-off ", status=202)
        else:
            response = flask.Response(remote_response_content, status=409)
        
    except ValueError as err:
        return flask.Response(str(err), status=409)

    return response

@parent_server.route('/takeoff_and_land', methods=['GET'])
def takeoff_land():
    response = ''
    try:
        response = takeoff()
        gevent.sleep(2)
        response = land()
        
    except ValueError as err:
        return flask.Response(str(err), status=409)

    return response

@parent_server.route('/land', methods=['GET'])
def land():
    response = ''
    remote_response_content = ''
    
    try:
        start_landing_ok = True
        for conf in child_server_ips:
            response = send_land(conf[0], conf[1])
            remote_response_content += str(response.content)
            
            if not response.status_code == 202:
                start_landing_ok = False
            
        
        if start_landing_ok:
            response = flask.Response("Landing", status=202)
        else:
            response = flask.Response(remote_response_content, status=409)
            
    except ValueError as err:
        return flask.Response(str(err), status=409)

    return response


@parent_server.route('/stop', methods=['GET'])
def stop():
    response = ''
    try:
        for conf in child_server_ips:
            response = response + str(send_stop(conf[0], conf[1]).content)  
    except ValueError as err:
        return flask.Response(str(err), status=409)

    return flask.Response(status=202)

@parent_server.route('/restart', methods=['GET'])
def restart():
    response = ''
    try:
        for conf in child_server_ips:
            response = response + str(send_restart(conf[0], conf[1]).content)  
    except ValueError as err:
        return flask.Response(str(err), status=409)

    return flask.Response(status=202)

@parent_server.route('/shutdown', methods=['GET'])
def shutdown():
    response = ''
    try:
        for conf in child_server_ips:
            response = response + str(send_shutdown(conf[0], conf[1]).content)  
    except ValueError as err:
        return flask.Response(str(err), status=409)

    return flask.Response(status=202)


@parent_server.route('/status', methods=['GET'])
def get_status():
    child_status = ''
    for conf in child_server_ips:
        result = get_status_child_servers(conf[0], conf[1])
        child_status += str(result.content)
        
    parent_status = 'NO_INFORMATION'
    if child_status.count("IDLE") == 2:
        parent_status = "IDLE"
    elif child_status.count("LAUNCHING") == 2:
        parent_status = 'LAUNCHING' 
    elif child_status.count("READY") == 2:
        parent_status = 'READY' 
    elif child_status.count("FLYING") == 2:
        parent_status = 'FLYING' 
    elif child_status.count("STOPPING") == 2:
        parent_status = 'STOPPING' 
    
#     return parent_status
    return flask.Response(parent_status, status=200)

@parent_server.route('/drones_status', methods=['GET'])
def get_drone_status():
    drone_status = get_drone_status_ping()
    return flask.jsonify(drone_status)


def send_start_flying(child_server_ip, child_server_port):
    response = requests.post(
        'http://' + str(child_server_ip) + ':' + str(child_server_port) + '/start-flying')
    return response

def send_restart(child_server_ip, child_server_port):
    response = requests.post(
        'http://' + str(child_server_ip) + ':' + str(child_server_port) + '/restart')
    return response

def send_shutdown(child_server_ip, child_server_port):
    response = requests.post(
        'http://' + str(child_server_ip) + ':' + str(child_server_port) + '/shutdown')
    return response


def send_stop(child_server_ip, child_server_port):
    response = requests.post(
        'http://' + str(child_server_ip) + ':' + str(child_server_port) + '/stop')
    return response

def send_takeoff(child_server_ip, child_server_port):
    response = requests.post(
        'http://' + str(child_server_ip) + ':' + str(child_server_port) + '/takeoff')
    return response

def send_land(child_server_ip, child_server_port):
    response = requests.post(
        'http://' + str(child_server_ip) + ':' + str(child_server_port) + '/land')
    return response


def get_status_child_servers(child_server_ip, child_server_port):
    response = requests.get(
        'http://' + str(child_server_ip) + ':' + str(child_server_port) + '/status')
    return response

def get_drone_status_ping():
    drone_status = Pinger.ping_all_drones(drone_ips)
#     drone_status = dict()
#     drone_status["Nerve"] = 0
#     drone_status["Juliet"] = 0
#     drone_status["Romeo"] = 1
#     drone_status["Fievel"] = 0
#     drone_status["Dumbo"] = 1
    return drone_status
    
# def persist_config(config_dir, drone_ips):
#     with open("drones_config.yaml", 'w') as stream:
#         try:
#             conf_file = yaml.load(stream)
#         except yaml.YAMLError as exc:
#             print(exc)
#
#     conf_file.dump()


if __name__ == '__main__':
    global child_server_ips
    global drone_ips

    host_ip = str(sys.argv[1])
    port = int(sys.argv[2])
    if len(sys.argv) == 4:
        child_server_ips = [('0.0.0.0', 8080)]

    parent_server.run(host=host_ip, port=port, threaded=True)
