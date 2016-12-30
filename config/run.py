"""
Scripts to bootstrap a rats show.
"""

import atexit
import datetime
import os
import multiprocessing

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
    if 'pose_aggregation' in main_configs:
        executor.start_pose_aggregation(main_configs['pose_aggregation'], tracker,
                                        log_dir + '/pose_aggregation', config_dir)

    test_xbox_controller()
    if 'synchronizer' in main_configs:
        executor.start_synchronizer(main_configs['synchronizer'], tracker,
                                    log_dir + '/synchronizer', config_dir)

    # to keep the script alive
    print('Start flying!! Press ENTER to terminate the program.')
    input()


def start_bebops(bebop_configs, launch_components, tracker, log_dir, config_dir):
    configs.check_unique_integer_id(bebop_configs)
    # iterate over all bebops
    processes = []
    for bebop, config in bebop_configs.items():
        p = multiprocessing.Process(target=start_single_bebop, args=(
            tracker, bebop, config, launch_components, log_dir + '/' + bebop, config_dir))
        processes.append(p)
        p.start()

    for p in processes:
        p.join()


def start_single_bebop(tracker, bebop_name, config, launch_components, log_dir, config_dir):
    print('------------------------' + bebop_name + '-------------------------')

    my_env = create_env(config['ros_master_port'])
    executor.launch_ros_master(my_env, config['ros_master_port'], config['sync_config'], tracker,
                               config_dir, log_dir)

    executor.load_ros_parameters_from_file(config_dir + '/beswarm.yaml', my_env, log_dir)
    executor.load_ros_parameters_from_file(config_dir + '/arlocros.yaml', my_env, log_dir)
    executor.set_ros_parameters(my_env, config['rosparam'], log_dir)

    if launch_components['bebop_autonomy']:
        executor.launch_bebop_autonomy(config['bebop_ip'], my_env, tracker, log_dir)

    if launch_components['point_camera_downward']:
        executor.point_camera_downward(my_env, tracker, log_dir)

    if launch_components['record_rosbag']:
        executor.record_rosbag(my_env, tracker, log_dir)

    if launch_components['launch_xbox_controller']:
        executor.launch_xbox_controller(my_env, tracker, log_dir)

    if launch_components['launch_arlocros']:
        executor.launch_arlocros(my_env, tracker, log_dir)

    if launch_components['launch_beswarm']:
        executor.launch_beswarm(my_env, tracker, config['beswarm_config'], log_dir)

    if 'topic_relay' in config:
        executor.relay_topics(my_env, config['topic_relay'], bebop_name, tracker, log_dir)


def test_xbox_controller():
    print("TEST YOUR XBOX CONTROLLER, PRESS ENTER WHEN YOU ARE READY!")
    input()


def create_env(port):
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
    return my_env


if __name__ == '__main__':
    start()
