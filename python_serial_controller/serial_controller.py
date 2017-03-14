## SERIAL CONTROLLER TCP BRIDGE
# Author: Andres Rodriguez Reina
# Mar 19 2017
#
# USAGE:
# Robust retry system for serial to TCP connection python script
# The script retries to connect serial or TCP whenever connection is
# lost (edit port and serial port list below)

import argparse
import errno
import time
import socket
import sys
import os
import serial

def connection_handler(connection,callback):
    connection.setblocking(0)
    # Receive the data in lines
    while True:
        try:
            data = connection.recv(4096)
            print >>sys.stderr, 'connection_handler: received "%s"' % data
            if data:
                #process data
                connection.sendall(callback(data))
            else:
                break
        except socket.error as e:
            if(e.errno==errno.EWOULDBLOCK):
                time.sleep(0.01)
                s=callback(0)
                if(s):
                    print >>sys.stderr, 'connection_handler: received data from serial to tcp: ',s
                    connection.sendall(s)
            else:
                raise

def start_unix_server(callback, server_address = '\0serial_socket\0'):
    #notify callback no connections
    callback(-1)
    while True:
        try:
            # Make sure the socket does not already exist
            try:
                os.unlink(server_address)
            except OSError:
                if os.path.exists(server_address):
                    raise

            # Create a UDS socket
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

            # Bind the socket to the port
            print >>sys.stderr, 'unix_server: starting up on %s' % server_address
            sock.bind(server_address)

            # Listen for incoming connections
            sock.listen(1)

            while True:
                # Wait for a connection
                print >>sys.stderr, 'unix_server: waiting for a connection'
                connection, client_address = sock.accept()
                try:
                    print >>sys.stderr, 'unix_server: connection from', client_address
                    connection_handler(connection,callback)
                    print >>sys.stderr, 'connection_handler: no more data from', client_address
                except:
                    raise
                finally:
                    # Clean up the connection
                    connection.close()

                    
            sock.close()
        except Exception as e:
            print >>sys.stderr, '******** EXCEPTION: unix_server: ',str(e)
            try:
                sock.close()
            except Exception:
                pass
        
        print >>sys.stderr, 'unix_server: retrying ... '
        #notify callback error happened
        callback(-1)
        time.sleep(1)  
        #notify callback no connections
        callback(-1)

def start_tcp_server(callback, port = 8000):
    #notify callback no connections
    callback(-1)
    while True:
        if True: #try:

            # Create a UDS socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
            #reuse address if in use
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # Bind the socket to the port
            print >>sys.stderr, 'tcp_server: starting up on %s:%s' % (socket.gethostname(),port)
            sock.bind(('', port))

            # Listen for incoming connections
            sock.listen(1)
            sock.settimeout(1)

            while True:
                # Wait for a connection
                print >>sys.stderr, 'tcp_server: waiting for a connection'
                while True:
                    try:
                        connection, client_address = sock.accept()
                        break
                    except socket.timeout:
                        #notify callback disconnect happened
                        callback(-1)
                try:
                    print >>sys.stderr, 'tcp_server: connection from', client_address
                    try:
                        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 1)
                        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
                        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
                        connection.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                    except AttributeError:
                        pass # XXX not available on windows
                    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    connection_handler(connection,callback)
                    print >>sys.stderr, 'connection_handler: no more data from', client_address
                except:
                    raise 
                finally:
                    # Clean up the connection
                    connection.close()
                #notify callback disconnect happened
                callback(-1)

                    
            sock.close()
        #except Exception as e:
            #print >>sys.stderr, '******** EXCEPTION: tcp_server: ',str(e)
            #try:
                #sock.close()
            #except Exception:
                #pass
        
        print >>sys.stderr, 'tcp_server: retrying ... '
        #notify callback error happened
        callback(-1)
        time.sleep(1)  
        #notify callback no connections
        callback(-1)

def callback(data):
    if data==-2:
        return #nothing is to be done
    if data==-1: #error - send safe command
        return callback(serial_connect.err_str)
    try: #just send stuff
        if data==0: #get serial data
            data = callback.serial.read(1024)
            return data
        else:
            print >>sys.stderr, 'serial_write: writting data: ', data
            callback.serial.write(data)
            callback.serial.flush()
            return ''
    except Exception as e:
        print >>sys.stderr, '******** EXCEPTION: reading/sending through serial: ',str(e)
        callback.serial=serial_connect(callback.serial)
        callback(data)
    return ''

def serial_connect(old_serial):
    #disconnect
    try:
        old_serial.close()
    except:
        pass
    #try forever
    while True:
        for port in serial_connect.ports:
            print >>sys.stderr, 'serial_connect: trying ',port
            try:
                if serial_connect.maxon:
                    s=serial.Serial(port, 115200, bytesize=serial.EIGHTBITS,
                             parity=serial.PARITY_NONE,
                             stopbits=serial.STOPBITS_ONE,
                             timeout=0,
                             xonxoff=0,
                             rtscts=0)
                else:
                    s=serial.Serial(port,serial_connect.baudrate,timeout=0)
                print >>sys.stderr, 'serial_connect: connected ',port
                return s
            except Exception as e:
                print >>sys.stderr, '******** EXCEPTION: serial_connect: ',str(e)
            time.sleep(0.5)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-b","--baudrate",help="baudrate to use",default=115200)
    parser.add_argument("-s","--serialport",help="serial port to use",action='append')
    parser.add_argument("-p","--tcpport",help="tcp port to listen to",default=8000,type=int)
    parser.add_argument("-M","--maxon",help="maxon serial port configuration",action='store_true',default=True,dest='maxon')
    parser.add_argument("-m","--nomaxon",help="use standard serial port configuration",action='store_false',default=True,dest='maxon')
    parser.add_argument("-e","--error",help="pattern to send through serial port on error",default="!G 1 0_!G 2 0_")
    parser.add_argument("-n","--ignore_error",help="do not send any pattern through serial port on error",action='store_true',default=False)
    args=parser.parse_args()
    #print args
    serial_connect.baudrate=args.baudrate
    serial_connect.ports=args.serialport
    if not serial_connect.ports:
        serial_connect.ports=["/dev/ttyACM0","/dev/ttyACM1","/dev/ttyACM2","/dev/ttyACM3","/dev/ttyUSB0","/dev/ttyUSB1","/dev/ttyUSB2"]
    serial_connect.maxon=args.maxon
    if not args.ignore_error:
        serial_connect.err_str=args.error+"\n"
    else:
        serial_connect.err_str=-1
    callback.serial=serial_connect(None)
    while True:
        try:
            start_tcp_server(callback,args.tcpport)
        except Exception as e:
            print >>sys.stderr, '******** EXCEPTION: setting up server: ',str(e)
        print >>sys.stderr, 'server: retrying ... '
        #notify callback error happened
        callback(-1)
        time.sleep(1)  
        #notify callback no connections
        callback(-1)
