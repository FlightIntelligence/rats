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
import sys
import datetime


def start():
    """
    Main entrance of the scripts.
    """
    main_config_file_name = sys.argv[1]
    # get the current path of the script
    current_path = os.path.dirname(os.path.realpath(__file__))
    # parse the main config file
    parsed_config_file = parse_yaml_file(current_path + '/' + main_config_file_name)
    # convert the parsed config file to python dictionary
    configs = read_yaml_file(parsed_config_file)

    # the list of all active processes
    processes = []
    # clean up on exit
    atexit.register(clean_up, processes)

    # here we go
    print('Start the program...')

    log_dir_abs_path = os.path.expanduser('~') + '/run_script_log/' + datetime.datetime.now(
    ).strftime("%Y-%m-%d-%H-%M-%S") + '/'

    start_bebops(configs['bebops'], processes, log_dir_abs_path)
    test_xbox_controller()
    start_synchronizer(configs['synchronizer'], processes, log_dir_abs_path + 'synchronizer')

    # to keep the script alive
    input()


def start_bebops(bebop_configs, processes, log_dir_abs_path):
    # iterate over all bebops
    for bebop, config in bebop_configs.items():
        # start a bebop using her own config
        start_single_bebop(
            process_list=processes,
            config=config,
            log_file_prefix_abs_path=log_dir_abs_path + bebop)


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


def start_single_bebop(process_list, config, log_file_prefix_abs_path):
    """
    Starts a single bebop.
    :param process_list: the list of active processes
    :type process_list: list
    :param config: the configuration of the bebop
    :type config: dict
    """
    my_env = create_env(config['local_drone_ip'], config['port'])
    launch_ros_master(my_env, config['port'], process_list, config['master_sync_config_file'],
                      log_file_prefix_abs_path)
    launch_bebop_autonomy(config['bebop_ip'], my_env, process_list,
                          log_file_prefix_abs_path + '_launch_bebop_autonomy.log')
    point_camera_downward(my_env, process_list,
                          log_file_prefix_abs_path + '_point_camera_downward.log')
    record_rosbag(my_env, process_list, log_file_prefix_abs_path + '_record_rosbag.log')
    launch_xbox_controller(my_env, process_list,
                           log_file_prefix_abs_path + '_launch_xbog_controller.log')
    launch_arlocros(my_env, process_list, config['arlocros_config_file'],
                    log_file_prefix_abs_path + '_launch_arlocros.log')
    launch_beswarm(my_env, process_list, config['beswarm_config'], log_file_prefix_abs_path)


def test_xbox_controller():
    print("TEST YOUR XBOX CONTROLLER, PRESS ENTER WHEN YOU ARE READY!")
    input()


def start_synchronizer(synchronizer_config, process_list, log_file_prefix_abs_path):
    my_env = os.environ.copy()
    my_env['ROS_MASTER_URI'] = 'http://localhost:' + synchronizer_config['port']
    launch_ros_master(my_env, synchronizer_config['port'], process_list,
                      synchronizer_config['master_sync_config_file'], log_file_prefix_abs_path)
    set_ros_parameters(my_env, process_list, synchronizer_config['rosparam'],
                       log_file_prefix_abs_path + '_set_rosparam.log')
    synchronizer_launch_cmd = 'rosrun rats ' + synchronizer_config[
        'python_node'] + ' >> ' + log_file_prefix_abs_path + '_launch_synchronizer'
    process_list.append(subprocess.Popen(synchronizer_launch_cmd.split(), env=my_env))
    time.sleep(2)


def launch_beswarm(my_env, process_list, beswarm_config, log_file_prefix_abs_path):
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
    load_param_cmd = 'rosparam load ' + parsed_beswarm_config_file + ' >> ' + log_file_prefix_abs_path + '_rosparam_load.log'
    process_list.append(subprocess.Popen(load_param_cmd.split(), env=my_env))
    time.sleep(2)
    # set some remaining parameters to the parameter server
    set_ros_parameters(my_env, process_list, beswarm_config['rosparam'],
                       log_file_prefix_abs_path + '_set_rosparam.log')
    time.sleep(2)
    # launch the java node
    beswarm_launch_cmd = 'rosrun rats BeSwarm ' + beswarm_config[
        'javanode'] + ' __name:=BeSwarm >> ' + log_file_prefix_abs_path + '_launch_beswarm.log'
    process_list.append(subprocess.Popen(beswarm_launch_cmd.split(), env=my_env))
    time.sleep(2)


def launch_xbox_controller(my_env, process_list, log_file_abs_path):
    launch_xbox_controller_cmd = 'roslaunch bebop_tools joy_teleop.launch joy_config:=xbox360 >> ' + log_file_abs_path
    process_list.append(subprocess.Popen(launch_xbox_controller_cmd.split(), env=my_env))


def record_rosbag(my_env, process_list, log_file_abs_path):
    record_rosbag_cmd = 'rosbag record /bebop/image_raw /bebop/cmd_vel /bebop/odom /tf /bebop/camera_info /arlocros/pose >> ' + log_file_abs_path
    process_list.append(subprocess.Popen(record_rosbag_cmd.split(), env=my_env))


def set_ros_parameters(my_env, process_list, ros_params, log_file_abs_path):
    """
    Sets the ros parameters to the parameter server.
    :param my_env: the environment
    :type my_env: _Environ
    :param process_list: the list of active processes
    :type process_list: list
    :param ros_params: the parameters to be set to the parameter server. It is a dictionary with
    keys being parameters name and values being parameter values
    :type ros_params: dict
    :param log_file_abs_path: the absolute path of the log file
    """
    for key, value in ros_params.items():
        set_param_cmd = 'rosparam set ' + str(key) + ' ' + str(value) + ' >> ' + log_file_abs_path
        process_list.append(subprocess.Popen(set_param_cmd.split(), env=my_env))


def launch_arlocros(my_env, process_list, arlocros_config_file_abs_path, log_file_prefix_abs_path):
    # parse the configuration file and load it to the parameter server
    parsed_arlocros_config_file = parse_yaml_file(arlocros_config_file_abs_path)
    load_param_cmd = 'rosparam load ' + parsed_arlocros_config_file + ' >> ' + log_file_prefix_abs_path + '_rosparam_load.log'
    process_list.append(subprocess.Popen(load_param_cmd.split(), env=my_env))
    time.sleep(2)
    # launch the java node
    arlocros_launch_cmd = 'rosrun rats ARLocROS arlocros.ARLoc __name:=ARLocROS >> ' + log_file_prefix_abs_path + '_arlocros_launch.log'
    process_list.append(subprocess.Popen(arlocros_launch_cmd.split(), env=my_env))
    time.sleep(2)


def launch_bebop_autonomy(bebop_ip, my_env, process_list, log_file_abs_path):
    bebop_launch_cmd = 'roslaunch bebop_driver bebop_node.launch ip:=' + bebop_ip + ' >> ' + log_file_abs_path
    process_list.append(subprocess.Popen(bebop_launch_cmd.split(), env=my_env))
    time.sleep(2)


def launch_ros_master(my_env, port, process_list, master_sync_config_file_abs_path,
                      log_file_prefix_abs_path):
    # start a ros master
    roscore_cmd = 'roscore -p ' + port + ' >> ' + log_file_prefix_abs_path + '_roscore.log'
    process_list.append(subprocess.Popen(roscore_cmd.split(), env=my_env))
    time.sleep(2)
    # start master_discovery_fkie (to discover other ros masters)
    master_discovery_cmd = 'rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.1 >> ' + log_file_prefix_abs_path + '_master_discovery.log'
    process_list.append(subprocess.Popen(master_discovery_cmd.split(), env=my_env))
    time.sleep(2)
    # start master_sync_fkie (to sync with other ros masters
    parsed_master_sync_config_file = parse_yaml_file(master_sync_config_file_abs_path)
    sync_cmd = 'rosrun master_sync_fkie master_sync _interface_url:=' + \
               parsed_master_sync_config_file + ' >> ' + log_file_prefix_abs_path + '_sync_cmd.log'
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


def point_camera_downward(my_env, process_list, log_file_abs_path):
    point_camera_cmd = 'rostopic pub /bebop/camera_control geometry_msgs/Twist "[0.0,0.0,0.0]" "[0.0,-50.0,0.0]" >> ' + log_file_abs_path
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
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
    print('cleaned up')


def remove_tmp_files():
    """
    Removes all generated tmp files.
    """
    for file_name in glob.glob('./*_tmp.yaml'):
        os.remove(file_name)


if __name__ == '__main__':
    start()
