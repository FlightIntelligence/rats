"""
Scripts to bootstrap a rats show.
"""

import subprocess
import os
import atexit
import signal
import time
import yaml
import glob


def start():
    """
    Main entrance of the scripts.
    """
    # get the current path of the script
    current_path = os.path.dirname(os.path.realpath(__file__))
    # parse the main config file
    parsed_config_file = parse_yaml_file(current_path + '/config.yaml')
    # convert the parsed config file to python dictionary
    configs = read_yaml_file(parsed_config_file)

    # the list of all active processes
    processes = []
    # clean up on exit
    atexit.register(clean_up, processes)

    # here we go
    print('Start the program...')

    start_bebops(configs['bebops'], processes)
    start_synchronizer(configs['synchronizer'], processes)

    # to keep the script alive
    input()


def start_bebops(bebop_configs, processes):
    # iterate over all bebops
    for bebop, config in bebop_configs.items():
        # start a bebop using her own config
        start_single_bebop(process_list=processes, config=config)


def read_yaml_file(yaml_file):
    """
    Reads a yaml file and translate it to a python dictionary.
    :param yaml_file: the absolute directory of the yaml file to be read
    :type yaml_file: str
    :return: the python dictionary representing the content of the yaml file
    :rtype: dict
    """
    with open(yaml_file, 'r') as stream:
        try:
            return yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)


def parse_yaml_file(yaml_file):
    """
    Parses a yaml file. A new temporary yaml file will be generated based on the input file with
    all the substitution arguments replaced by their values.
    :param yaml_file: the absolute directory of the yaml file to be parsed
    :type yaml_file: str
    :return: the absolute directory of the new temporary yaml file
    :rtype: str
    """
    file = open(yaml_file, 'r')
    content = file.read()
    file.close()

    # replace the absolute path variables
    content = content.replace('${abs_path}', os.path.dirname(os.path.realpath(__file__)))

    # read all other variables
    variables = read_yaml_file(os.path.dirname(os.path.realpath(__file__)) + '/variables.yaml')

    # replace all substitution arguments/variables by their values defined in variables.yaml
    for key, value in variables.items():
        content = content.replace('${' + key + '}', value)

    # check if there is any substitution argument not being defined in variables.yaml
    index = content.find('${')
    if index != -1:
        print('Cannot parse ' + yaml_file + '. Variable ' + content[index:content.find('}') + 1] +
              ' is not defined in variables.yaml.')
        # if such variable exists, exit immediately
        exit()

    # the absolute directory of the temporary file
    parsed_file = yaml_file.replace('.yaml', '_tmp.yaml')

    # write the temporary file to disk
    tmp_file = open(parsed_file, 'w')
    tmp_file.write(content)
    tmp_file.close()

    # return the absolute directory of the temporary file
    return parsed_file


def start_single_bebop(process_list, config):
    """
    Starts a single bebop.
    :param process_list: the list of active processes
    :type process_list: list
    :param config: the configuration of the bebop
    :type config: dict
    """
    # initialize the environment
    my_env = create_env(config['local_drone_ip'], config['port'])
    # launch the ros master
    launch_ros_master(my_env, config['port'], process_list, config['master_sync_config_file'])
    # launch the bebop_autonomy
    launch_bebop_autonomy(config['bebop_ip'], my_env, process_list)
    # point the camera downward
    point_camera_downward(my_env, process_list)
    # record rosbag
    record_rosbag(my_env, process_list)
    # launch bebop xbox controller
    launch_xbox_controller(my_env, process_list)
    # launch ARLocROS
    launch_arlocros(my_env, process_list, config['arlocros_config_file'])
    # launch BeSwarm
    launch_beswarm(my_env, process_list, config['beswarm_config'])


def start_synchronizer(synchronizer_config, process_list):
    my_env = os.environ.copy()
    my_env['ROS_MASTER_URI'] = 'http://localhost:' + synchronizer_config['port']
    launch_ros_master(my_env, synchronizer_config['port'], process_list,
                      synchronizer_config['master_sync_config_file'])
    set_ros_parameters(my_env, process_list, synchronizer_config['rosparam'])
    synchronizer_launch_cmd = 'rosrun rats BeSwarm ' + synchronizer_config[
        'javanode'] + ' __name:=Synchronizer'
    process_list.append(subprocess.Popen(synchronizer_launch_cmd.split(), env=my_env))
    time.sleep(2)


def launch_xbox_controller(my_env, process_list):
    launch_xbox_controller_cmd = 'roslaunch bebop_tools joy_teleop.launch joy_config:=xbox360'
    process_list.append(subprocess.Popen(launch_xbox_controller_cmd.split(), env=my_env))


def record_rosbag(my_env, process_list):
    record_rosbag_cmd = 'rosbag record /bebop/image_raw /bebop/cmd_vel /bebop/odom /tf /bebop/camera_info /arlocros/pose'
    process_list.append(subprocess.Popen(record_rosbag_cmd.split(), env=my_env))


def set_ros_parameters(my_env, process_list, ros_params):
    """
    Sets the ros parameters to the parameter server.
    :param my_env: the environment
    :type my_env: _Environ
    :param process_list: the list of active processes
    :type process_list: list
    :param ros_params: the parameters to be set to the parameter server. It is a dictionary with
    keys being parameters name and values being parameter values
    :type ros_params: dict
    """
    for key, value in ros_params.items():
        set_param_cmd = 'rosparam set ' + key + ' ' + value
        process_list.append(subprocess.Popen(set_param_cmd.split(), env=my_env))


def launch_beswarm(my_env, process_list, beswarm_config):
    """
    Launches BeSwarm.
    :param my_env: the environment
    :type my_env: _Environ
    :param process_list: the list of active processes
    :type process_list: list
    :param beswarm_config: the configuration of BeSwarm
    :type beswarm_config: dict
    """
    # parse the beswarm config file and load it to the parameter server
    parsed_beswarm_config_file = parse_yaml_file(beswarm_config['beswarm_config_file'])
    load_param_cmd = 'rosparam load ' + parsed_beswarm_config_file
    process_list.append(subprocess.Popen(load_param_cmd.split(), env=my_env))
    time.sleep(2)
    # set some remaining parameters to the parameter server
    set_ros_parameters(my_env, process_list, beswarm_config['rosparam'])
    time.sleep(2)
    # launch the java node
    beswarm_launch_cmd = 'rosrun rats BeSwarm ' + beswarm_config['javanode'] + ' __name:=BeSwarm'
    process_list.append(subprocess.Popen(beswarm_launch_cmd.split(), env=my_env))
    time.sleep(2)


def launch_arlocros(my_env, process_list, arlocros_config_file):
    """
    Launches ARLocROS
    :param my_env: the environment
    :type my_env: _Environ
    :param process_list: the list of active processes
    :type process_list: list
    :param arlocros_config_file: the absolute path of the configuration file for ARLocROS
    :type arlocros_config_file: str
    """
    # parse the configuration file and load it to the parameter server
    parsed_arlocros_config_file = parse_yaml_file(arlocros_config_file)
    load_param_cmd = 'rosparam load ' + parsed_arlocros_config_file
    process_list.append(subprocess.Popen(load_param_cmd.split(), env=my_env))
    time.sleep(2)
    # launch the java node
    arlocros_launch_cmd = 'rosrun rats ARLocROS arlocros.ARLoc __name:=ARLocROS'
    process_list.append(subprocess.Popen(arlocros_launch_cmd.split(), env=my_env))
    time.sleep(2)


def launch_bebop_autonomy(bebop_ip, my_env, process_list):
    """
    Launches the bebop autonomy
    :param bebop_ip: the ip of the bebop
    :type bebop_ip: str
    :param my_env: the environment
    :type my_env: _Environ
    :param process_list: the list of active processes
    :type process_list: list
    """
    bebop_launch_cmd = 'roslaunch bebop_driver bebop_node.launch ip:=' + bebop_ip
    process_list.append(subprocess.Popen(bebop_launch_cmd.split(), env=my_env))
    time.sleep(2)


def launch_ros_master(my_env, port, process_list, master_sync_config_file):
    """
    Launches a ros master and fkie packages.
    :param my_env: the environment
    :type my_env: _Environ
    :param port: the port of the master
    :type port: str
    :param process_list: the list of active processes
    :type process_list: list
    :param master_sync_config_file: the absolute path of the configuration file for the
    master_sync_fkie package
    :type master_sync_config_file: str
    """
    # start a ros master
    process_list.append(subprocess.Popen(['roscore', '-p', port], env=my_env))
    time.sleep(2)
    # start master_discovery_fkie (to discover other ros masters)
    master_discovery_cmd = 'rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.1'
    process_list.append(subprocess.Popen(master_discovery_cmd.split(), env=my_env))
    time.sleep(2)
    # start master_sync_fkie (to sync with other ros masters
    parsed_master_sync_config_file = parse_yaml_file(master_sync_config_file)
    sync_cmd = 'rosrun master_sync_fkie master_sync _interface_url:=' + \
               parsed_master_sync_config_file
    process_list.append(subprocess.Popen(sync_cmd.split(), env=my_env))
    time.sleep(2)


def create_env(local_drone_ip, port):
    """
    Creates a local environment.
    :param local_drone_ip: the ip of the network adapter connected to the drone
    :type local_drone_ip: str
    :param port: the port of the ros master
    :type port: str
    """
    my_env = os.environ.copy()
    my_env['ROS_MASTER_URI'] = 'http://localhost:' + port
    my_env['LOCAL_DRONE_IP'] = local_drone_ip
    return my_env


def point_camera_downward(my_env, process_list):
    """
    Point the front camera downward
    :param my_env: the environment
    :type my_env: _Environ
    :param process_list: the list of active processes
    :type process_list: list
    """
    point_camera_cmd = 'rostopic pub /bebop/camera_control geometry_msgs/Twist "[0.0,0.0,0.0]" "[0.0,-50.0,0.0]"'
    process_list.append(subprocess.Popen(point_camera_cmd.split(), env=my_env))


def clean_up(processes):
    """
    Cleans up on exit.
    :param processes: the list of active processes
    :type processes: list
    """
    # remove all generated tmp files
    remove_tmp_files()
    # terminate all processes
    terminate_all_processes(processes)


def terminate_all_processes(processes):
    """
    Terminates all processes.
    :param processes: the list of active processes
    :type processes: list
    """
    for p in processes:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    print('cleaned up')


def remove_tmp_files():
    """
    Removes all generated tmp files.
    """
    for file_name in glob.glob('./*_tmp.yaml'):
        os.remove(file_name)


if __name__ == '__main__':
    start()
