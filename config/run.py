import subprocess
import os
import atexit
import signal
import time
import yaml
import glob


def start():
    current_path = os.path.dirname(os.path.realpath(__file__))
    parsed_config_file = parse_yaml_file(current_path + '/config.yaml')
    configs = read_yaml_file(parsed_config_file)

    processes = []
    atexit.register(clean_up, processes)
    print('Start the program...')

    for key, value in configs.items():
        start_single_master(process_list=processes, config=value)

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

    # replace the absolute path variables
    content = content.replace('${abs_path}', os.path.dirname(os.path.realpath(__file__)))

    # read all other variables
    variables = read_yaml_file(os.path.dirname(os.path.realpath(__file__)) + '/variables.yaml')

    for key, value in variables.items():
        content = content.replace('${' + key + '}', value)

    index = content.find('${')
    if index != -1:
        print('Cannot parse ' + yaml_file + '. Variable ' + content[index:content.find('}') + 1] +
              ' is not defined in variables.yaml.')
        exit()

    parsed_file = yaml_file.replace('.yaml', '_tmp.yaml')
    tmp_file = open(parsed_file, 'w')
    tmp_file.write(content)
    tmp_file.close()

    return parsed_file


def start_single_master(process_list, config):
    my_env = create_env(config['local_drone_ip'], config['port'])
    launch_ros_master(my_env, config['port'], process_list, config['master_sync_config_file'])
    launch_bebop_autonomy(config['bebop_ip'], my_env, process_list)
    launch_arlocros(my_env, process_list, config['arlocros_config_file'])


def launch_arlocros(my_env, process_list, arlocros_config_file):
    parsed_arlocros_config_file = parse_yaml_file(arlocros_config_file)
    load_param_cmd = 'rosparam load ' + parsed_arlocros_config_file
    process_list.append(subprocess.Popen(load_param_cmd.split(), env=my_env))
    time.sleep(2)
    arlocros_launch_cmd = 'rosrun rats ARLocROS arlocros.ARLoc __name:=ARLocROS'
    process_list.append(subprocess.Popen(arlocros_launch_cmd.split(), env=my_env))
    time.sleep(2)


def launch_bebop_autonomy(bebop_ip, my_env, process_list):
    bebop_launch_cmd = 'roslaunch bebop_driver bebop_node.launch ip:=' + bebop_ip
    process_list.append(subprocess.Popen(bebop_launch_cmd.split(), env=my_env))
    time.sleep(2)


def launch_ros_master(my_env, port, process_list, master_sync_config_file):
    process_list.append(subprocess.Popen(['roscore', '-p', port], env=my_env))
    time.sleep(2)
    master_discovery_cmd = 'rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.1'
    process_list.append(subprocess.Popen(master_discovery_cmd.split(), env=my_env))
    time.sleep(2)
    parsed_master_sync_config_file = parse_yaml_file(master_sync_config_file)
    sync_cmd = 'rosrun master_sync_fkie master_sync _interface_url:=' + parsed_master_sync_config_file
    process_list.append(subprocess.Popen(sync_cmd.split(), env=my_env))
    time.sleep(2)


def create_env(local_drone_ip, port):
    my_env = os.environ.copy()
    my_env['ROS_MASTER_URI'] = 'http://localhost:' + port
    my_env['LOCAL_DRONE_IP'] = local_drone_ip
    return my_env


def clean_up(processes):
    remove_tmp_files()
    terminate_all_processes(processes)


def terminate_all_processes(processes):
    for p in processes:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    print('cleaned up')


def remove_tmp_files():
    for file_name in glob.glob('./*_tmp.yaml'):
        os.remove(file_name)


if __name__ == '__main__':
    start()
