import os

import configs


def ping(host):
    response = os.system('ping -c 5 ' + host)
    return response


def ping_all_bebops(bebop_configs):
    for bebop, config in bebop_configs.items():
        response = ping(config['bebop_ip'])
        print(bebop)
        print(response)


if __name__ == '__main__':
    config_dir = configs.get_config_dir()
    main_configs = configs.get_main_config(config_dir)
    ping_all_bebops(main_configs['bebops'])
