from distutils import dir_util
import uuid
import os
import time

import signal
import yaml
from SwarmBootstrapUtils import yaml_parser
import subprocess
import enum
import multiprocessing


class Launcher:
    def __init__(self):
        self._run_process = None
        self._status = Launcher.Status.IDLE
        self._status_file = 'status.txt'

    @staticmethod
    def _clone_config_folder(original_folder_dir):
        original_folder_name = original_folder_dir.split('/')[-1]
        current_dir = os.path.dirname(os.path.realpath(__file__))
        generated_folder_dir = current_dir + '/' + original_folder_name + '_tmp_' + str(
            uuid.uuid4())
        dir_util.copy_tree(original_folder_dir, generated_folder_dir)
        return generated_folder_dir

    @staticmethod
    def _replace_drone_ip(config_dir, drone_ips):
        config_file_dir = config_dir + '/config.yaml'
        config = yaml_parser.read(config_file_dir)
        for drone_name, ip in drone_ips.items():
            if drone_name in config['bebops']:
                config['bebops'][drone_name]['bebop_ip'] = ip
            else:
                raise IndexError('There is no drone named ' + drone_name + ' in config.yaml')
        config_file = open(config_file_dir, 'w')
        yaml.dump(config, config_file, default_flow_style=False)

    def _start_script(self, config_dir, drone_ips):
        cloned_config_dir = self._clone_config_folder(config_dir)
        self._replace_drone_ip(cloned_config_dir, drone_ips)
        run_cmd = 'python3 run.py ' + cloned_config_dir
        self._run_process = subprocess.Popen(run_cmd.split(), start_new_session=True,
                                             stdin=subprocess.PIPE)

    def _stop_script(self):
        pgid = os.getpgid(self._run_process.pid)
        os.killpg(pgid, signal.SIGINT)
        os.waitpid(-pgid, 0)
        alive_pgids = subprocess.check_output('ps x o pgid'.split()).decode(
            "utf-8").rstrip().replace(' ', '').split('\n')
        while str(pgid) in alive_pgids:
            time.sleep(1)
            alive_pgids = subprocess.check_output('ps x o pgid'.split()).decode(
                "utf-8").rstrip().replace(' ', '').split('\n')
        self._status = Launcher.Status.IDLE
        self._run_process = None

    def _wait_for_ready(self):
        while True:
            next_line = self._run_process.stdout.readline().decode("utf-8").rstrip()
            if next_line == 'TEST YOUR XBOX CONTROLLER, PRESS ENTER WHEN YOU ARE READY!':
                self._status = Launcher.Status.READY
                return

    def launch(self, config_dir, drone_ips):
        if self._status == Launcher.Status.IDLE:
            self._status = Launcher.Status.LAUNCHING
            self._start_script(config_dir, drone_ips)
            wait_process = multiprocessing.Process(target=self._wait_for_ready)
            wait_process.start()
        else:
            raise ValueError(
                'Script can only be launched if current state is IDLE, but the current state is: '
                '' + str(self._status.name))

    def start_flying(self):
        if self._status == Launcher.Status.READY:
            self._status = Launcher.Status.FLYING
            self._run_process.communicate(b'\n')
        else:
            raise ValueError(
                'Can only start flying if the current state is READY, but the current state is: '
                + str(
                    self._status.name))

    def stop(self):
        if self._status == Launcher.Status.STOPPING:
            raise ValueError('Waiting for all processes being killed. Please be patient...')
        elif self._status == Launcher.Status.IDLE:
            raise ValueError('There is no process running')
        else:
            self._status = Launcher.Status.STOPPING
            stopping_process = multiprocessing.Process(target=self._stop_script)
            stopping_process.start()

    def _write_state_to_file(self, state):
        with open(self._status_file, 'a+') as file:
            file.write(str(state.name))

    def _read_last_state_from_file(self):
        with open(self._status_file) as file:
            # The last line is a blank line. We read the second last one
            last_state = file.readlines()[-2]
        return last_state

    class Status(enum.Enum):
        IDLE = 1
        LAUNCHING = 2
        READY = 3
        FLYING = 4
        STOPPING = 5
