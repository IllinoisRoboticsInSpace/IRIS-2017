#!/bin/bash
./test | python controller_to_net.py -a 192.168.7.112:9001 -m 192.168.7.112:9002
