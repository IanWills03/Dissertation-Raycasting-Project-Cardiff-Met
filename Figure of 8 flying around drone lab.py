import socket
import cv2
import time
from djitellopy import Tello
#from Samples.PythonClient.NatNetClient import NatNetClient
#from Samples.PythonClient.MoCapData import MoCapData
from time import sleep
from threading import Thread
import numpy as np
import time


# Tello Command Function
print("start")
TELLO_IP = '192.168.10.1'
TELLO_PORT = 8889
#TELLO_VIDEO_PORT = 11111
bufferSize = 1024
# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
tello_address = (TELLO_IP, TELLO_PORT)

def patrolling():
    msg = send_command('forward 200')
    print(msg)
    msg = send_command('curve 50 50 0 0 100 0 30')
    print(msg)
    msg = send_command('cw 180')
    print(msg)
    msg = send_command('forward 220')
    print(msg)
    msg = send_command('curve 50 -50 0 0 -100 0 30')
    print(msg)
    msg = send_command('cw 180')

# Send command to Tello
def send_command(command):
    sock.sendto(command.encode('utf-8'), tello_address)
    msg = sock.recvfrom(bufferSize)
    return msg
print("start commands")
msg = send_command("command")
print(msg)
msg = send_command('battery?')
print(msg)
msg = send_command('takeoff')
print(msg)
print("Patrolling the room...")
msg = send_command('up 30')
print(msg)
patrolling()
patrolling()
patrolling()

msg = send_command('land')
print(msg)
msg = send_command('reboot')
print(msg)


