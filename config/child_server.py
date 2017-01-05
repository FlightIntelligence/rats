import flask
from flask import json
import backend

launcher = backend.Launcher()

child_server = flask.Flask(__name__)


@child_server.route('/', methods=['POST'])
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


@child_server.route('/', methods=['POST'])
def start_flying():
    try:
        launcher.start_flying()
    except ValueError as err:
        return flask.Response(str(err), status=409)

    return flask.Response(status=202)


@child_server.route('/', methods=['POST'])
def stop():
    try:
        launcher.stop()
    except ValueError as err:
        return flask.Response(str(err), status=409)

    return flask.Response(status=202)


@child_server.route('/', methods=['GET'])
def get_status():
    return str(launcher.get_status())


if __name__ == '__main__':
    child_server.run(host='127.0.0.1', port=8001, threaded=True)
