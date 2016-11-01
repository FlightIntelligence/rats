"""
Scripts to bootstrap a rats show.
"""

import atexit
import datetime
import os
import shutil
import subprocess
import time

from SwarmBootstrapUtils import configs
from SwarmBootstrapUtils import yaml_parser

from SwarmBootstrapUtils import clean_up


def start():
    config_dir = configs.get_config_dir()
    main_configs = configs.get_main_config(config_dir)

    # the list of all active processes
    tracker = {'processes': [], 'opened_files': []}
    # clean up on exit
    atexit.register(clean_up.clean_up, tracker['processes'], tracker['opened_files'], config_dir)

    # here we go
    print('Start the program...')

    log_dir = os.path.expanduser('~') + '/log_rats/' + datetime.datetime.now().strftime(
        "%Y-%m-%d-%H-%M-%S")

    start_bebops(main_configs['bebops'], main_configs['launch_components'], tracker, log_dir,
                 config_dir)
    test_xbox_controller()
    start_synchronizer(main_configs['synchronizer'], tracker, log_dir + '/synchronizer', config_dir)

    # to keep the script alive
    input()


def start_bebops(bebop_configs, launch_components, tracker, log_dir, config_dir):
    # iterate over all bebops
    for bebop, config in bebop_configs.items():
        # start a bebop using her own config
        start_single_bebop(tracker=tracker, config=config, launch_components=launch_components,
                           log_dir=log_dir + '/' + bebop, config_dir=config_dir)


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
        substituted_beswarm_config_file = yaml_parser.substitute(beswarm_config_file)
        load_param_cmd = 'rosparam load ' + substituted_beswarm_config_file
        execute_cmd(load_param_cmd, my_env, log_dir + '/rosparam_load.log', tracker)
        time.sleep(2)
        # set some remaining parameters to the parameter server
        set_ros_parameters(my_env, tracker, beswarm_config['rosparam'], log_dir)
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
                        '/bebop/camera_info /arlocros/marker_pose /arlocros/fused_pose'
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
        substituted_arlocros_config_file = yaml_parser.substitute(arlocros_config_file)
        load_param_cmd = 'rosparam load ' + substituted_arlocros_config_file
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
        substituted_master_sync_config_file = yaml_parser.substitute(master_sync_config_file)
        sync_cmd = 'rosrun master_sync_fkie master_sync _interface_url:=' + \
                   substituted_master_sync_config_file
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
