"""
Scripts to bootstrap a rats show.
"""

import atexit
import datetime
import os

from SwarmBootstrapUtils import configs
from SwarmBootstrapUtils import clean_up
from SwarmBootstrapUtils import executor


def start():
    log_dir = os.path.expanduser('~') + '/log_rats/' + datetime.datetime.now().strftime(
        "%Y-%m-%d-%H-%M-%S")

    configs.log_commit_numbers(log_dir)

    config_dir = configs.get_config_dir()
    config_dir = configs.copy_config_to_log_dir(config_dir, log_dir)
    main_configs = configs.get_main_config(config_dir)

    # the list of all active processes
    tracker = {'processes': [], 'opened_files': []}
    # clean up on exit
    atexit.register(clean_up.clean_up, tracker['processes'], tracker['opened_files'])

    # here we go
    print('Start the program...')

    start_bebops(main_configs['bebops'], main_configs['launch_components'], tracker, log_dir,
                 config_dir)
    test_xbox_controller()
    if 'synchronizer' in main_configs:
        executor.start_synchronizer(main_configs['synchronizer'], tracker,
                                    log_dir + '/synchronizer', config_dir)

    # to keep the script alive
    input()


def start_bebops(bebop_configs, launch_components, tracker, log_dir, config_dir):
    configs.check_unique_integer_id(bebop_configs)
    # iterate over all bebops
    for bebop, config in bebop_configs.items():
        # start a bebop using her own config
        start_single_bebop(tracker=tracker, config=config, launch_components=launch_components,
                           log_dir=log_dir + '/' + bebop, config_dir=config_dir)


def start_single_bebop(tracker, config, launch_components, log_dir, config_dir):
    my_env = create_env(config['local_drone_ip'], config['ros_master_port'])
    executor.launch_ros_master(my_env, config['ros_master_port'], tracker, config_dir, log_dir)

    if launch_components['bebop_autonomy']:
        executor.launch_bebop_autonomy(config['bebop_ip'], my_env, tracker, log_dir)

    if launch_components['point_camera_downward']:
        executor.point_camera_downward(my_env, tracker, log_dir)

    if launch_components['record_rosbag']:
        executor.record_rosbag(my_env, tracker, log_dir)

    if launch_components['launch_xbox_controller']:
        executor.launch_xbox_controller(my_env, tracker, log_dir)

    if launch_components['launch_arlocros']:
        executor.launch_arlocros(my_env, tracker, config_dir, log_dir)

    if launch_components['launch_beswarm']:
        executor.launch_beswarm(my_env, tracker, config['beswarm_config'], config_dir, log_dir)


def test_xbox_controller():
    print("TEST YOUR XBOX CONTROLLER, PRESS ENTER WHEN YOU ARE READY!")
    input()


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


if __name__ == '__main__':
    start()
