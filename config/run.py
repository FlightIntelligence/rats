import subprocess
import os
import atexit
import signal
import time


def start():
    sync_config = os.path.dirname(os.path.realpath(__file__)) + '/sync_config.yaml'
    processes = []
    print('Start the program...')

    start_single_master(processes, '11311', sync_config, '192.168.32.1', '192.168.32.23',
        'markerPingPongExtended.cfg', 'false')

    start_single_master(processes, '11312', sync_config, '192.168.42.1', '192.168.42.87',
        'markerPingPongExtended.cfg', 'false')

    atexit.register(clean_up, processes)
    input()


def start_single_master(processes, port, sync_config, bebop_ip, local_drone_ip, marker_config_file,
        use_threshold):
    my_env = os.environ.copy()
    my_env['ROS_MASTER_URI'] = 'http://localhost:' + port
    my_env['LOCAL_DRONE_IP'] = local_drone_ip
    processes.append(subprocess.Popen(['roscore', '-p', port], env=my_env))
    time.sleep(2)
    master_discovery_cmd = 'rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.1'
    processes.append(subprocess.Popen(master_discovery_cmd.split(), env=my_env))
    time.sleep(2)
    sync_cmd = 'rosrun master_sync_fkie master_sync _interface_url:=' + sync_config
    processes.append(subprocess.Popen(sync_cmd.split(), env=my_env))
    time.sleep(2)
    bebop_launch_cmd = 'roslaunch bebop_driver bebop_node.launch ip:=' + bebop_ip
    processes.append(subprocess.Popen(bebop_launch_cmd.split(), env=my_env))
    time.sleep(2)
    arlocros_launch_cmd = 'roslaunch rats bebop.launch marker_config_file:=' + \
                          marker_config_file + ' use_threshold:=' + use_threshold
    processes.append(subprocess.Popen(arlocros_launch_cmd.split(), env=my_env))


def clean_up(processes):
    for p in processes:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    print('cleaned up')


if __name__ == '__main__':
    start()

