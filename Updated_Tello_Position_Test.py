import threading
import time
import socket
from MotionCapture_Files.NatNetClient import NatNetClient

# ---------------------------
# Shared variable for drone position (updated by Motive/NatNet)
drone_position = {"x": 0.0, "y": 0.0, "z": 0.0}
lock = threading.Lock()

# Callback function to process all rigid body data, but only track the Tello's position
def receive_rigid_body_position(rigid_body_id, position, rotation):
    global drone_position
    with lock:
        drone_position["x"], drone_position["y"], drone_position["z"] = position

# Thread 1: Continuously runs the NatNet client to receive position data.
def receive_position():
    client = NatNetClient()
    client.rigid_body_listener = receive_rigid_body_position
    client.run()

# Thread 2: Only prints the drone's updated position every 0.1 seconds

def display_position():
    last_position = {"x": 0.0, "y": 0.0, "z": 0.0}
    while True:
        with lock:
            pos = drone_position.copy()
        if pos != last_position:
            print(f"Drone Position: x={pos['x']:.3f}, y={pos['y']:.3f}, z={pos['z']:.3f}")
            last_position = pos
        time.sleep(0.1)

# ---------------------------
# Tello Drone Control Setup using UDP socket
TELLO_IP = '192.168.10.1'
TELLO_PORT = 8889
BUFFER_SIZE = 1024
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
tello_address = (TELLO_IP, TELLO_PORT)

# Function to send commands to Tello and print its response.
def send_command(command):
    print(f"Sending command: {command}")
    sock.sendto(command.encode('utf-8'), tello_address)
    try:
        msg, _ = sock.recvfrom(BUFFER_SIZE)
        print(f"Response: {msg.decode('utf-8')}")
        return msg
    except socket.timeout:
        print("No response from Tello")
        return None

# Tello flight formation (patrolling pattern)
def patrolling():

    commands = [
        'forward 180',
        'curve 50 50 0 0 100 0 30',
        'cw 180',
        'forward 180',
        'curve 50 -50 0 0 -100 0 20',
        'cw 180'
    ]
    for cmd in commands:
        msg = send_command(cmd)
        time.sleep(2)

# Function that sequences the Tello commands

def tello_startup():
    print("Initializing Tello command mode...")
    send_command("command")
    send_command("battery?")
    send_command("takeoff")
    send_command("up 30")

    print("Patrolling the room...")
    for i in range(2):
        print(f"Patrol iteration {i + 1}")
        patrolling()

    send_command("land")
    send_command("reboot")

# ---------------------------
# Start threads to receive and display NatNet data
natnet_thread = threading.Thread(target=receive_position, daemon=True)
display_thread = threading.Thread(target=display_position, daemon=True)
natnet_thread.start()
display_thread.start()

# Wait for NatNet data to initialize
time.sleep(5)

# Run Tello commands in main thread
tello_startup()

# Keep the script running to allow display updates
while True:
    time.sleep(1)
