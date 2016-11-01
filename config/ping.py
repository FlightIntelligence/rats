import os

from src import configs


def ping(host):
    response = os.system('ping -c 5 ' + host)
    return response


def ping_all_bebops(bebop_configs):
    for bebop, config in bebop_configs.items():
        print('-----------' + bebop + '-----------')
        response = ping(config['bebop_ip'])
        if response == 0:
            print(bebop + ' is ok.')
        else:
            print(bebop + ' is down!!!')
        print('\n\n\n')


if __name__ == '__main__':
    config_dir = configs.get_config_dir()
    main_configs = configs.get_main_config(config_dir)
    ping_all_bebops(main_configs['bebops'])
