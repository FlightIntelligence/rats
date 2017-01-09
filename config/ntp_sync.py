import os
import subprocess
import sys
import time

if __name__ == '__main__':
    host = sys.argv[1]

    print("Start ntp loader")
    while True:
        response = os.system('ping -c 1 ' + host)
        if response == 0:
            print("Ping successfully")
            break
        print("Cannot ping " + str(host) + "yet.")
        time.sleep(1)

    subprocess.call('service ntp stop'.split())
    subprocess.call('ntpd -gq'.split())
    subprocess.call('service ntp start'.split())
    print("Time synced!")
