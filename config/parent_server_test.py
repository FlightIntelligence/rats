import parent_server


parent_server.child_server_ips = [('0.0.0.0', 8080),
                    ('0.0.0.0', 8081)]


if __name__ == '__main__':

    while True:
        usr_input = int(input('Press 1 to launch, 2 to start flying, 3 to stop, 4 to get status, 5 to get config, 6 takeoff : '))
        if usr_input == 1:
            parent_server.start_launch()
        elif usr_input == 2:
            parent_server.start_flying()
        elif usr_input == 3:
            parent_server.stop()
        elif usr_input == 4:
            print(parent_server.get_status())
        elif usr_input == 5:
            print(parent_server.get_config())
