"""
Scripts to bootstrap a rats show.
"""

import subprocess
import os
import atexit
import signal
import time

import shutil
import yaml
import glob
import sys
import datetime


def start():
    config_dir = get_config_dir()
    configs = get_main_config(config_dir)

    # the list of all active processes
    tracker = {'processes': [], 'opened_files': []}
    # clean up on exit
    atexit.register(clean_up, tracker, config_dir)

    # here we go
    print('Start the program...')

    log_dir = os.path.expanduser('~') + '/log_rats/' + datetime.datetime.now().strftime(
        "%Y-%m-%d-%H-%M-%S")

    start_bebops(configs['bebops'], configs['launch_components'], tracker, log_dir, config_dir)
    test_xbox_controller()
    start_synchronizer(configs['synchronizer'], tracker, log_dir + '/synchronizer', config_dir)

    # to keep the script alive
    input()


def get_main_config(config_dir):
    config_file = config_dir + '/config.yaml'

    if os.path.isfile(config_file):
        # parse the main config file
        parsed_config_file = parse_yaml_file(config_file)
        # convert the parsed config file to python dictionary
        configs = read_yaml_file(parsed_config_file)
        return configs
    else:
        print('FILE NOT FOUND: ', config_file)
        exit()


def get_config_dir():
    try:
        config_dir = sys.argv[1]
    except IndexError:
        print('Please pass the absolute path of the configuration folder')
        exit()

    if config_dir[-1] == '/':
        config_dir = config_dir[:-1]

    if os.path.isdir(config_dir):
        return config_dir
    else:
        print(config_dir, ' is not a valid directory')
        exit()


def start_bebops(bebop_configs, launch_components, tracker, log_dir, config_dir):
    # iterate over all bebops
    for bebop, config in bebop_configs.items():
        # start a bebop using her own config
        start_single_bebop(tracker=tracker, config=config, launch_components=launch_components,
                           log_dir=log_dir + '/' + bebop, config_dir=config_dir)


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

    config_dir = os.path.dirname(yaml_file)
    # replace the absolute path variables
    content = content.replace('${config_dir}', config_dir)

    # read all other variables
    variables = read_yaml_file(config_dir + '/variables.yaml')

    # replace all substitution arguments/variables by their values defined in variables.yaml
    for key, value in variables.items():
        content = content.replace('${' + key + '}', value)

    # check if there is any substitution argument not being defined in variables.yaml
    index = content.find('${')
    if index != -1:
        print('Cannot parse ' + yaml_file + '. Variable ' + content[index:content.find(
            '}') + 1] + ' is not defined in variables.yaml.')
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


def start_single_bebop(tracker, config, launch_components, log_dir, config_dir):
    my_env = create_env(config['local_drone_ip'], config['port'])
    launch_ros_master(my_env, config['port'], tracker, config_dir, log_dir)

    if launch_components['bebop_autonomy']:
        launch_bebop_autonomy(config['bebop_ip'], my_env, tracker, log_dir)

    if launch_components['point_camera_downward']:
        point_camera_downward(my_env, tracker, log_dir)

    if launch_components['record_rosbag']:
        record_rosbag(my_env, tracker, log_dir)

    if launch_components['launch_xbox_controller']:
        launch_xbox_controller(my_env, tracker, log_dir)

    if launch_components['launch_arlocros']:
        launch_arlocros(my_env, tracker, config_dir, log_dir)

    if launch_components['launch_beswarm']:
        launch_beswarm(my_env, tracker, config['beswarm_config'], config_dir, log_dir)


def test_xbox_controller():
    print("TEST YOUR XBOX CONTROLLER, PRESS ENTER WHEN YOU ARE READY!")
    input()


def start_synchronizer(synchronizer_config, tracker, log_dir, config_dir):
    my_env = os.environ.copy()
    my_env['ROS_IP'] = '127.0.0.1'
    my_env['ROS_MASTER_URI'] = 'http://localhost:' + synchronizer_config['port']
    launch_ros_master(my_env, synchronizer_config['port'], tracker, config_dir, log_dir)
    set_ros_parameters(my_env, tracker, synchronizer_config['rosparam'], log_dir)
    synchronizer_launch_cmd = 'rosrun rats ' + synchronizer_config['python_node']
    execute_cmd(synchronizer_launch_cmd, my_env, log_dir + '/launch_synchronizer.log', tracker)
    time.sleep(2)


def launch_beswarm(my_env, tracker, beswarm_config, config_dir, log_dir):
    beswarm_config_file = config_dir + '/beswarm.yaml'
    my_env['LOG_DIR'] = log_dir

    if os.path.isfile(beswarm_config_file):
        # delete build script folder
        build_script_dir = execute_cmd_and_get_output(
            'rospack find rats') + '/BeSwarm/build/scripts'
        shutil.rmtree(build_script_dir, ignore_errors=True)
        # parse the beswarm config file and load it to the parameter server
        parsed_beswarm_config_file = parse_yaml_file(beswarm_config_file)
        load_param_cmd = 'rosparam load ' + parsed_beswarm_config_file
        execute_cmd(load_param_cmd, my_env, log_dir + '/rosparam_load.log', tracker)
        time.sleep(2)
        # set some remaining parameters to the parameter server
        set_ros_parameters(my_env, tracker, beswarm_config['rosparam'],
                           log_dir + '/set_rosparam.log')
        time.sleep(2)
        # launch the java node
        beswarm_launch_cmd = 'rosrun rats BeSwarm ' + beswarm_config['javanode'] + ' __name:=' + \
                             beswarm_config['nodename']
        execute_cmd(beswarm_launch_cmd, my_env, log_dir + '/launch_beswarm.log', tracker)
        time.sleep(2)
    else:
        print('FILE NOT FOUND: ', beswarm_config_file)
        exit()


def launch_xbox_controller(my_env, tracker, log_dir):
    launch_xbox_controller_cmd = 'roslaunch bebop_tools joy_teleop.launch joy_config:=xbox360'
    execute_cmd(launch_xbox_controller_cmd, my_env, log_dir + '/launch_xbox.log', tracker)


def record_rosbag(my_env, tracker, log_dir):
    record_rosbag_cmd = 'rosbag record /bebop/image_raw /bebop/cmd_vel /bebop/odom /tf ' \
                        '/bebop/camera_info /arlocros/pose'
    execute_cmd(record_rosbag_cmd, my_env, log_dir + '/record_rosbag.log', tracker)


def set_ros_parameters(my_env, tracker, ros_params, log_dir):
    log_file = log_dir + '/set_rosparam.log'
    for key, value in ros_params.items():
        set_param_cmd = 'rosparam set ' + str(key) + ' ' + str(value)
        execute_cmd(set_param_cmd, my_env, log_file, tracker)


def launch_arlocros(my_env, tracker, config_dir, log_dir):
    arlocros_config_file = config_dir + '/arlocros.yaml'

    if os.path.isfile(arlocros_config_file):
        # delete build script folder
        build_script_dir = execute_cmd_and_get_output(
            'rospack find rats') + '/ARLocROS/build/scripts'
        shutil.rmtree(build_script_dir, ignore_errors=True)
        # parse the configuration file and load it to the parameter server
        parsed_arlocros_config_file = parse_yaml_file(arlocros_config_file)
        load_param_cmd = 'rosparam load ' + parsed_arlocros_config_file
        execute_cmd(load_param_cmd, my_env, log_dir + '/rosparam_load.log', tracker)
        time.sleep(2)
        # launch the java node
        arlocros_launch_cmd = 'rosrun rats ARLocROS arlocros.ARLoc __name:=ARLocROS'
        execute_cmd(arlocros_launch_cmd, my_env, log_dir + '/launch_arlocros.log', tracker)
        time.sleep(2)
    else:
        print('FILE NOT FOUND: ', arlocros_config_file)


def launch_bebop_autonomy(bebop_ip, my_env, tracker, log_dir):
    bebop_launch_cmd = 'roslaunch bebop_driver bebop_node.launch ip:=' + bebop_ip
    execute_cmd(bebop_launch_cmd, my_env, log_dir + '/bebop_autonomy.log', tracker)
    time.sleep(2)


def launch_ros_master(my_env, port, tracker, config_dir, log_dir):
    master_sync_config_file = config_dir + '/master_sync.yaml'
    if os.path.isfile(master_sync_config_file):
        # start a ros master
        roscore_cmd = 'roscore -p ' + port
        execute_cmd(roscore_cmd, my_env, log_dir + '/roscore.log', tracker)
        time.sleep(2)
        # start master_discovery_fkie (to discover other ros masters)
        master_discovery_cmd = 'rosrun master_discovery_fkie master_discovery ' \
                               '_mcast_group:=224.0.0.1'
        execute_cmd(master_discovery_cmd, my_env, log_dir + '/master_discovery.log', tracker)
        time.sleep(2)
        # start master_sync_fkie (to sync with other ros masters
        parsed_master_sync_config_file = parse_yaml_file(master_sync_config_file)
        sync_cmd = 'rosrun master_sync_fkie master_sync _interface_url:=' + \
                   parsed_master_sync_config_file
        execute_cmd(sync_cmd, my_env, log_dir + '/sync_cmd.log', tracker)
        time.sleep(2)
    else:
        print('FILE NOT FOUND: ', master_sync_config_file)
        exit()


def create_env(local_drone_ip, port):
    """
    Creates a local environment.
    :param local_drone_ip: the ip of the network adapter connected to the drone
    :type local_drone_ip: str
    :param port: the port of the ros master
    :type port: str
    """
    my_env = os.environ.copy()
    my_env['ROS_IP'] = '127.0.0.1'
    my_env['ROS_MASTER_URI'] = 'http://localhost:' + port
    my_env['LOCAL_DRONE_IP'] = local_drone_ip
    return my_env


def point_camera_downward(my_env, tracker, log_dir):
    point_camera_cmd = 'rostopic pub /bebop/camera_control geometry_msgs/Twist [0.0,0.0,' \
                       '0.0] [0.0,-50.0,0.0]'
    execute_cmd(point_camera_cmd, my_env, log_dir + '/point_cam_downward.log', tracker)


def clean_up(tracker, config_dir):
    """
    Cleans up on exit.
    :param tracker: the list of active processes
    :type tracker: dict
    """
    # remove all generated tmp files
    remove_tmp_files(config_dir)
    # terminate all processes
    terminate_all_processes(tracker['processes'])
    close_all_opened_files(tracker['opened_files'])


def terminate_all_processes(processes):
    """
    Terminates all processes.
    :param processes: the list of active processes
    :type processes: list
    """
    for p in processes:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except KeyboardInterrupt:
            pass
    print('cleaned up')


def close_all_opened_files(opened_files):
    for f in opened_files:
        f.flush()
        f.close()


def remove_tmp_files(config_dir):
    """
    Removes all generated tmp files.
    """
    for file_name in glob.glob(config_dir + '/*_tmp.yaml'):
        os.remove(file_name)


def execute_cmd(cmd, my_env, log_file_abs_path, tracker):
    print(cmd)
    os.makedirs(os.path.dirname(log_file_abs_path), exist_ok=True)
    log_file = open(log_file_abs_path, 'a+')
    tracker['processes'].append(
        subprocess.Popen(cmd.split(), env=my_env, stdout=log_file, stderr=subprocess.STDOUT))
    tracker['opened_files'].append(log_file)


def execute_cmd_and_get_output(cmd):
    print(cmd)
    return subprocess.check_output(cmd.split()).decode("utf-8").replace('\n', '')


if __name__ == '__main__':
    start()
