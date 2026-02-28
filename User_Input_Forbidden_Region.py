import threading
import time
import socket
from MotionCapture_Files.NatNetClient import NatNetClient

# ---------------------------
# Shared variable for drone position (updated by Motive/NatNet)
drone_position = {"x": 0.0, "y": 0.0, "z": 0.0}
lock = threading.Lock()

# ---------------------------
# Raycasting: Point inside polygon

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

# ---------------------------
# Callback function to process rigid body data

def receive_rigid_body_position(rigid_body_id, position, rotation):
    global drone_position
    with lock:
        drone_position["x"], drone_position["y"], drone_position["z"] = position

# ---------------------------
# User-defined forbidden region input

def get_forbidden_polygon():
    print("\nINSTRUCTIONS:")
    print("You're entering forbidden region coordinates based on MOTION CAPTURE SPACE.")
    print("This is a TOP-DOWN VIEW: X = forward, Y = right (Z in OptiTrack).")
    print("Make sure your values reflect where the forbidden region is in the real room.\n")

    polygon = []
    print("Enter coordinates for forbidden polygon in meters (the coordinates must lie between -2 and 2) ")
    print("Type 'done' when finished")
    while True:
        user_input = input("Enter x y: ").strip()
        if user_input.lower() == 'done':
            break
        try:
            x_str, y_str = user_input.split()
            x, y = float(x_str), float(y_str)
            if not (-2 <= x <= 2 and -2 <= y <= 2):
                print("Coordinates must be between -2 and 2 meters")
                continue
            polygon.append((x, y))
        except ValueError:
            print("Invalid input. Please enter two numbers separated by a space.")
    return polygon

forbidden_polygon = get_forbidden_polygon()

# ---------------------------
# Tello UDP Setup
TELLO_IP = '192.168.10.1'
TELLO_PORT = 8889
BUFFER_SIZE = 1024
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
tello_address = (TELLO_IP, TELLO_PORT)
sock.settimeout(5)

# ---------------------------
# Command Sender

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

# ---------------------------
# Patrolling pattern

def patrolling():
    send_command("forward 250")
    send_command("ccw 90")
    send_command("forward 40")
    send_command("ccw 90")
    send_command("forward 250")
    send_command("cw 90")
    send_command("forward 40")
    send_command("cw 90")

# ---------------------------
# Position reader

def receive_position():
    client = NatNetClient()
    client.rigid_body_listener = receive_rigid_body_position
    client.run()

# ---------------------------
# Forbidden check

def display_position():
    last_position = {"x": 0.0, "y": 0.0, "z": 0.0}
    has_landed = False

    while True:
        with lock:
            pos = drone_position.copy()

        if pos != last_position:
            x_tello = pos['x']
            y_tello = pos['z']  # Top-down view: y is OptiTrack Z

            print(f"[Raycast] Checking drone position at x={x_tello:.3f}, y={y_tello:.3f}")
            if point_in_polygon(x_tello, y_tello, forbidden_polygon):
                print("\u26d4 Forbidden Region DETECTED!")
                if not has_landed:
                    print("\u26a0\ufe0f Executing emergency back 30 + landing sequence")
                    send_command("back 30")
                    time.sleep(2)
                    send_command("land")
                    has_landed = True
                    send_command("reboot")
            else:
                print("[Raycast] Safe — not in forbidden region.")

            last_position = pos
        time.sleep(0.5)

# ---------------------------
# Show polygon

def print_forbidden_region():
    print("\nForbidden Region (Raycasting Polygon in meters):")
    for i, (x, y) in enumerate(forbidden_polygon):
        print(f"  Vertex {i + 1}: x={x}, y={y}")

# ---------------------------
# Flight startup

def tello_startup():
    print("Initializing Tello command mode...")
    send_command("command")
    send_command("land")
    send_command("battery?")
    send_command("up 30")

    print("Patrolling the room...")
    for i in range(3):
        print(f"Patrol iteration {i + 1}")
        patrolling()

    send_command("land")
    send_command("reboot")

# ---------------------------
# START SYSTEM

# Start motion capture listener
natnet_thread = threading.Thread(target=receive_position, daemon=True)
display_thread = threading.Thread(target=display_position, daemon=True)
natnet_thread.start()
display_thread.start()

# Let it stabilize
time.sleep(5)
print_forbidden_region()

# Start drone sequence
tello_startup()

# Keep running
while True:
    time.sleep(1)
