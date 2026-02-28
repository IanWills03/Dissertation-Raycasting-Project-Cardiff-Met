import threading
import time
import socket
from MotionCapture_Files.NatNetClient import NatNetClient

# ---------------------------
# Shared variable for drone position (updated by Motive/NatNet)
drone_position = {"x": 0.0, "y": 0.0, "z": 0.0}
lock = threading.Lock()

# Define a forbidden region as a polygon (in cm)
forbidden_polygon = [
    (0.3, 0.3),  # bottom-left
    (1.3, 0.3),  # bottom-right
    (1.3, 1.3),  # top-right
    (0.3, 1.3)   # top-left
]

def point_in_polygon(x, y, polygon):
    """Ray casting algorithm to test if point is inside polygon."""
    num = len(polygon)
    j = num - 1
    inside = False
    for i in range(num):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-10) + xi):
            inside = not inside
        j = i
    return inside

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
    has_landed = False

    while True:
        with lock:
            pos = drone_position.copy()

        if pos != last_position:
            # Print only x and z (used for raycasting)
            print(f"Drone Position (Quanser frame): x={pos['x']:.3f}, z={pos['z']:.3f}")

            # Use x and z for 2D raycasting plane
            x_2d = pos['x']
            y_2d = pos['z']

            print(f"[Raycast] Checking 2D position at x={x_2d:.3f}, z={y_2d:.3f}")

            if point_in_polygon(x_2d, y_2d, forbidden_polygon) and not has_landed:
                print("⚠️ Forbidden region detected! Moving away and landing...")
                send_command("back 60")
                time.sleep(3)
                send_command("land")
                has_landed = True  # Prevents repeated actions
                send_command("reboot")

            last_position = pos

        time.sleep(0.5)



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
   send_command( "forward 220")
   send_command("ccw 90")
   send_command("forward 40")
   send_command("ccw 90")
   send_command("forward 220")
   send_command("cw 90")
   send_command("forward 40")
   send_command("cw 90")

#This code works on its own but when you encorparate the Motion capture system, the system is unable to
#update the drones position while it executes the curve command, making it not applicable to this project
   #send_command("curve 40 40 0 0 80 0 30")
   #send_command("cw 180")
   #send_command("forward 180")
   #send_command("curve 40 -40 0 0 -80 0 30")
   #send_command("cw 180")
   #time.sleep(2)



# Function that sequences the Tello commands

def print_forbidden_region():
    print("Forbidden Region (Raycasting Polygon in meters):")
    for i, (x, y) in enumerate(forbidden_polygon):
        print(f"  Vertex {i+1}: x={x}, y={y}")

def is_drone_in_forbidden_region():
    print("Waiting for valid drone position from motion capture system...")
    timeout = time.time() + 10  # 10-second timeout

    while True:
        with lock:
            x = drone_position["x"]
            z = drone_position["z"]
        if abs(x) > 0.01 or abs(z) > 0.01:
            break
        if time.time() > timeout:
            print("❌ Timeout waiting for drone position.")
            return True  # Fail-safe: assume inside to prevent unsafe behavior
        time.sleep(0.1)

    print(f"Starting drone position: x={x:.2f}, z={z:.2f}")
    return point_in_polygon(x, z, forbidden_polygon)

def tello_startup():
    print("Initializing Tello command mode...")
    send_command("command")
    send_command("battery?")
    send_command("takeoff")
    send_command("up 30")

    print("Patrolling the room...")
    for i in range(5):
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

# Print forbidden region details
print_forbidden_region()
if is_drone_in_forbidden_region():
    print("⚠️ Drone is in a forbidden region at startup. Aborting mission.")
    send_command("land")
    exit()
# Run Tello commands in main thread
tello_startup()






# Keep the script running to allow display updates
while True:
    time.sleep(1)
