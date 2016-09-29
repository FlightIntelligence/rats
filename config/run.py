import subprocess
import os
import atexit
import signal
import time
import yaml


def start():
    current_path = os.path.dirname(os.path.realpath(__file__))

    network_configs = read_yaml_file(current_path + '/network.yaml')
    master_sync_config_file = parse_yaml_file(current_path + '/master_sync.yaml')
    arlocros_config_file = parse_yaml_file(current_path + '/arlocros.yaml')

    processes = []
    print('Start the program...')

    for key, value in network_configs.items():
        start_single_master(
            process_list=processes,
            network_config=value,
            master_sync_config_file=master_sync_config_file,
            arlocros_config_file=arlocros_config_file)

    atexit.register(clean_up, processes)
    input()


def read_yaml_file(yaml_file):
    with open(yaml_file, 'r') as stream:
        try:
            return yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)


def parse_yaml_file(yaml_file):
    file = open(yaml_file, 'r')
    content = file.read()
    file.close()

    parsed_content = content.replace('${abs_path}', os.path.dirname(os.path.realpath(__file__)))

    parsed_file = yaml_file.replace('.yaml', '_tmp.yaml')
    tmp_file = open(parsed_file, 'w')
    tmp_file.write(parsed_content)
    tmp_file.close()

    return parsed_file


def start_single_master(process_list, network_config, master_sync_config_file,
                        arlocros_config_file):
    my_env = create_env(network_config['local_drone_ip'], network_config['port'])
    launch_ros_master(my_env, network_config['port'], process_list, master_sync_config_file)
    launch_bebop_autonomy(network_config['bebop_ip'], my_env, process_list)
    launch_arlocros(my_env, process_list, arlocros_config_file)


def launch_arlocros(my_env, process_list, arlocros_config_file):
    load_param_cmd = 'rosparam load ' + arlocros_config_file
    process_list.append(subprocess.Popen(load_param_cmd.split(), env=my_env))
    time.sleep(2)
    arlocros_launch_cmd = 'rosrun rats ARLocROS arlocros.ARLoc __name:=ARLocROS'
    process_list.append(subprocess.Popen(arlocros_launch_cmd.split(), env=my_env))
    time.sleep(2)


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
