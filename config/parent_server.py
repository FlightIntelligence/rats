import flask
import sys
from flask import json, render_template, url_for
import requests
import Pinger
import gevent

child_server_ips = [('192.168.13.108', 8080),
                    ('192.168.13.104', 8080)]

# config_dir = 0
# drone_ips = 0

drone_ips = dict()
drone_ips['Nerve'] = '192.168.13.170'
drone_ips['Romeo'] = '192.168.13.120'
drone_ips['Juliet'] = '192.168.13.110'
drone_ips['Fievel'] = '192.168.13.160'
drone_ips['Dumbo'] = '192.168.13.180'

config_dir = 'rats_show/full_show'

parent_server = flask.Flask(__name__)

@parent_server.route('/', methods=['POST', 'GET'])
def main():
    drones_info = dict()
    for k, v in drone_ips.items():
        temp = v.split(".")
        drones_info[k] = k + " " + temp[len(temp)-1]
         
    return flask.render_template('main_control.html', drone_ips=drones_info)

@parent_server.route('/set_config', methods=['POST'])
def set_config():
    global config_dir
    global drone_ips
    
    incorrect_form = False
    
    for k in drone_ips:
        if flask.request.form[k]:
            drone_ips[k] = flask.request.form[k]
        else:
            incorrect_form = True
    
    if flask.request.form['config_dir']:
        config_dir = flask.request.form['config_dir']
    else:
        incorrect_form = True
    
    if incorrect_form:
        return flask.Response('drone_ips not found', status=400)

#     persist_config(config_dir, drone_ips)
    return flask.Response("OK", status=202)

@parent_server.route('/config', methods=['GET'])
def config():
    global drone_ips
    global config_dir
    
    data = dict()
    data['drone_ips'] = drone_ips
    data['config_dir'] = config_dir
    
    return render_template('config.html', drone_ips = drone_ips)
#     return json.dumps(data)


@parent_server.route('/launch', methods=['GET'])
def start_launch():
    response = ''
    remote_response_content = ''
    
    try:
        launch_ok = True
        for conf in child_server_ips:
            response = send_launch_to_child_server(conf[0], conf[1])
            remote_response_content += str(response.content)
            
            if not response.status_code == 202:
                launch_ok = False
            
        
        if launch_ok:
            response = flask.Response("launching", status=202)
        else:
            response = flask.Response(remote_response_content, status=409)
            
    except ValueError as err:
        return flask.Response(str(err), status=409)

    return response


@parent_server.route('/start_flying', methods=['GET'])
def start_flying():
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

def send_launch_to_child_server(child_ip, child_port):
    global drone_ips
    global config_dir
    
    data = dict()
    data['drone_ips'] = drone_ips
    data['config_dir'] = config_dir

    json_data = json.dumps(data)

    response = requests.post(
        'http://' + str(child_ip) + ':' + str(child_port) + '/launch', json=json_data)
    return response


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
        child_server_ips = [('0.0.0.0', 8080),
                            ('0.0.0.0', 8081)]
        
        drone_ips = dict()
        drone_ips['Nerve'] = '127.0.0.1'
        drone_ips['Romeo'] = '127.0.0.1'
        drone_ips['Juliet'] = '8.8.8.8'
        drone_ips['Fievel'] = '127.0.0.1'
        drone_ips['Dumbo'] = '8.8.8.9'

     
    parent_server.run(host=host_ip, port=port, threaded=True)
