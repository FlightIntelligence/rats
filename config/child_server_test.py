import requests
from flask import json


def send_launch():
    drone_ips = dict()
    drone_ips['Nerve'] = '111.111.111.111'
    drone_ips['Romeo'] = '111.111.111.112'
    drone_ips['Juliet'] = '111.111.111.113'
    drone_ips['Fievel'] = '111.111.111.114'
    drone_ips['Dumbo'] = '111.111.111.115'

    config_dir = 'rats_show/full_show'

    data = dict()
    data['drone_ips'] = drone_ips
    data['config_dir'] = config_dir

    json_data = json.dumps(data)

    response = requests.post('http://192.168.0.103:8001/launch', json=json_data)
    print(response)


def send_start_flying():
    response = requests.post('http://192.168.0.103:8001/start-flying')
    print(response)


def send_stop():
    response = requests.post('http://192.168.0.103:8001/stop')
    print(response)


def get_status():
    response = requests.get('http://192.168.0.103:8001/status')
    print(response)


if __name__ == '__main__':
    while True:
        usr_input = int(input('Press 1 to launch, 2 to start flying, 3 to stop, 4 to get status: '))
        if usr_input == 1:
            send_launch()
        elif usr_input == 2:
            send_start_flying()
        elif usr_input == 3:
            send_stop()
        elif usr_input == 4:
            get_status()
