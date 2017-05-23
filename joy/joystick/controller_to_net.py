## SERIAL CONTROLLER TCP BRIDGE
# Author: Andres Rodriguez Reina
# May 2017
#

import argparse
import errno
import time
import socket
import sys
import os
import serial
import glob

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-m","--address_motor",help="address for motor tcp")
    parser.add_argument("-a","--address_actuators",help="address for actuators tcp")
    args=parser.parse_args()
    #!/usr/bin/env python

    a=args.address_motor.rsplit(":",1)
    print('connecting to %s for motor'%a)
    s_motor = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s_motor.connect((a[0],int(a[1])))
    a=args.address_actuators.rsplit(":",1)
    print('connecting to %s for actuators'%a)
    s_actuator = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s_actuator.connect((a[0],int(a[1])))

    #print args
    while 1:
        line=sys.stdin.readline()[:-1]
        if line[0]=='!':
            data=line.split(',')
            print('received %s'%data)
            c='!G 1 %s_!G 2 %s_\n'%tuple(data[0:2])
            print('sent motor %s'%c)
            s_motor.sendall(c)
            c='!%s,%s,%s,0#!\n'%tuple(data[2:5])
            print('sent actuator %s'%c)            
            s_actuator.sendall(c)
        else:
            print('error: missing initial ! in input')
        try:
            s_motor.setblocking(0)
            print('received motor: %s'%s_motor.recv(10000))
        except:
            pass        
        try:
            s_actuator.setblocking(0)
            print('received actuator: %s'%s_actuator.recv(10000))
        except:
            pass
