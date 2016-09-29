import subprocess
import os
import atexit
import signal
import time


def start():
    sync_config = os.path.dirname(os.path.realpath(__file__)) + '/sync_config.yaml'
    processes = []
    print('Start the program...')

    start_single_master(
        process_list=processes,
        port='11311',
        sync_config=sync_config,
        bebop_ip='192.168.32.1',
        local_drone_ip='192.168.32.23',
        marker_config_file='markerPingPongExtended.cfg',
        use_threshold='false')

    start_single_master(
        process_list=processes,
        port='11312',
        sync_config=sync_config,
        bebop_ip='192.168.42.1',
        local_drone_ip='192.168.42.87',
        marker_config_file='markerPingPongExtended.cfg',
        use_threshold='false')

    atexit.register(clean_up, processes)
    input()


def start_single_master(process_list, port, sync_config, bebop_ip, local_drone_ip,
                        marker_config_file, use_threshold):
    my_env = create_env(local_drone_ip, port)
    launch_ros_master(my_env, port, process_list, sync_config)
    launch_bebop_autonomy(bebop_ip, my_env, process_list)
    launch_arlocros(marker_config_file, my_env, process_list, use_threshold)


def launch_arlocros(marker_config_file, my_env, process_list, use_threshold):
    arlocros_launch_cmd = 'roslaunch rats bebop.launch marker_config_file:=' + marker_config_file\
                          + ' use_threshold:=' + use_threshold
    process_list.append(subprocess.Popen(arlocros_launch_cmd.split(), env=my_env))


def launch_bebop_autonomy(bebop_ip, my_env, process_list):
    bebop_launch_cmd = 'roslaunch bebop_driver bebop_node.launch ip:=' + bebop_ip
    process_list.append(subprocess.Popen(bebop_launch_cmd.split(), env=my_env))
    time.sleep(2)


def launch_ros_master(my_env, port, process_list, sync_config):
    process_list.append(subprocess.Popen(['roscore', '-p', port], env=my_env))
    time.sleep(2)
    master_discovery_cmd = 'rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.1'
    process_list.append(subprocess.Popen(master_discovery_cmd.split(), env=my_env))
    time.sleep(2)
    sync_cmd = 'rosrun master_sync_fkie master_sync _interface_url:=' + sync_config
    process_list.append(subprocess.Popen(sync_cmd.split(), env=my_env))
    time.sleep(2)


def create_env(local_drone_ip, port):
    my_env = os.environ.copy()
    my_env['ROS_MASTER_URI'] = 'http://localhost:' + port
    my_env['LOCAL_DRONE_IP'] = local_drone_ip
    return my_env


def clean_up(processes):
    for p in processes:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    print('cleaned up')


if __name__ == '__main__':
    start()
