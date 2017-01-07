import requests
from flask import json
import sys


def send_launch(child_server_ip, child_server_port):
    drone_ips = dict()
    drone_ips['Nerve'] = '192.168.13.170'
    drone_ips['Romeo'] = '192.168.13.120'
    drone_ips['Juliet'] = '192.168.13.110'
    drone_ips['Fievel'] = '192.168.13.160'
    drone_ips['Dumbo'] = '192.168.13.180'

    config_dir = 'rats_show/full_show'

    data = dict()
    data['drone_ips'] = drone_ips
    data['config_dir'] = config_dir

    json_data = json.dumps(data)

    response = requests.post(
        'http://' + str(child_server_ip) + ':' + str(child_server_port) + '/launch', json=json_data)
    print(response.content)


def send_start_flying(child_server_ip, child_server_port):
    response = requests.post(
        'http://' + str(child_server_ip) + ':' + str(child_server_port) + '/start-flying')
    print(response.content)


def send_stop(child_server_ip, child_server_port):
    response = requests.post(
        'http://' + str(child_server_ip) + ':' + str(child_server_port) + '/stop')
    print(response.content)


def get_status(child_server_ip, child_server_port):
    response = requests.get(
        'http://' + str(child_server_ip) + ':' + str(child_server_port) + '/status')
    print(response.content)


if __name__ == '__main__':
    child_server_ip = sys.argv[1]
    child_server_port = sys.argv[2]

    while True:
        usr_input = int(input('Press 1 to launch, 2 to start flying, 3 to stop, 4 to get status: '))
        if usr_input == 1:
            send_launch(child_server_ip, child_server_port)
        elif usr_input == 2:
            send_start_flying(child_server_ip, child_server_port)
        elif usr_input == 3:
            send_stop(child_server_ip, child_server_port)
        elif usr_input == 4:
            get_status(child_server_ip, child_server_port)
