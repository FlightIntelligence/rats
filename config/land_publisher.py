import requests
import threading
import enum
import time


class LandPublisher:
    def __init__(self):
        self._status = LandPublisher.Status.IDLE

    def _publish_land_message(self, child_server_ips):
        while self._status == LandPublisher.Status.RUNNING:
            self._land(child_server_ips)
            time.sleep(0.5)

        self._status = LandPublisher.Status.IDLE

    def start(self, child_server_ips):
        if self._status == LandPublisher.Status.IDLE:
            self._status = LandPublisher.Status.RUNNING
            publisher_thread = threading.Thread(target=self._publish_land_message,
                                                args=(child_server_ips,))
            publisher_thread.start()

    def stop(self):
        if self._status == LandPublisher.Status.RUNNING:
            self._status = LandPublisher.Status.STOPPING
            while self._status != LandPublisher.Status.IDLE:
                time.sleep(0.5)
            time.sleep(2)

    class Status(enum.Enum):
        IDLE = 1
        RUNNING = 2
        STOPPING = 3

    def _land(self, child_server_ips):
        remote_response_content = ''

        try:
            start_landing_ok = True
            for conf in child_server_ips:
                response = self._send_land(conf[0], conf[1])
                remote_response_content += str(response.content)

                if not response.status_code == 202:
                    start_landing_ok = False

            if start_landing_ok:
                response = 'ok'
            else:
                response = 'not ok'
        except ValueError:
            return 'super not ok'

        return response

    @staticmethod
    def _send_land(child_server_ip, child_server_port):
        response = requests.post(
            'http://' + str(child_server_ip) + ':' + str(child_server_port) + '/land')
        return response
