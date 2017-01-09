import ping3

def ping_all_drones(drone_ips):
    status = dict()
    for drone_name, ip in drone_ips.items():
        status[drone_name] = ping(ip)
    
    return status

def ping(host):
    ping_response = ping3.do_one(host, timeout=2)
    if (ping_response == None):
        return 1
    else:
        return 0
