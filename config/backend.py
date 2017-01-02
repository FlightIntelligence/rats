from distutils import dir_util
import uuid
import os
import time

import signal
import yaml
from SwarmBootstrapUtils import yaml_parser
import subprocess


class Launcher:
    def __init__(self, config_dir, drone_ips):
        self._config_dir = config_dir
        self._drone_ips = drone_ips
        self._run_process = None

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

    def start(self):
        if self._run_process is None:
            cloned_config_dir = self._clone_config_folder(self._config_dir)
            self._replace_drone_ip(cloned_config_dir, self._drone_ips)
            run_cmd = 'python3 run.py ' + cloned_config_dir
            self._run_process = subprocess.Popen(run_cmd.split(), start_new_session=True)

    def stop(self):
        if self._run_process is not None:
            pgid = os.getpgid(self._run_process.pid)
            os.killpg(pgid, signal.SIGINT)
            os.waitpid(-pgid, 0)
            alive_pgids = subprocess.check_output('ps x o pgid'.split()).decode(
                "utf-8").rstrip().replace(' ', '').split('\n')
            while str(pgid) in alive_pgids:
                time.sleep(1)
                alive_pgids = subprocess.check_output('ps x o pgid'.split()).decode(
                    "utf-8").rstrip().replace(' ', '').split('\n')
