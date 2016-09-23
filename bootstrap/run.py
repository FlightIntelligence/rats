import subprocess
import os
import atexit
import signal
import time


def start():
    sync_config = os.path.dirname(os.path.realpath(__file__)) + '/sync_config.yaml'
    processes = []
    print('Start the program...')

    start_single_master(processes, '11311', sync_config)
    start_single_master(processes, '11312', sync_config)

    atexit.register(clean_up, processes)
    input()


def start_single_master(processes, port, sync_config):
    os.environ['ROS_MASTER_URI'] = 'http://localhost:' + port
    processes.append(subprocess.Popen(['roscore', '-p', port]))
    time.sleep(2)
    processes.append(subprocess.Popen(
        'rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.1'.split()))
    time.sleep(2)
    sync_cmd = 'rosrun master_sync_fkie master_sync _interface_url:="' + sync_config + '"'
    processes.append(subprocess.Popen(sync_cmd.split()))


def clean_up(processes):
    for p in processes:
        print('kill smt')
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    print('cleaned up')


if __name__ == '__main__':
    start()

