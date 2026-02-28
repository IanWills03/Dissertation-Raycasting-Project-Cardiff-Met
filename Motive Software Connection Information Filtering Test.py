import threading
import time
from MotionCapture_Files.NatNetClient import NatNetClient

# Shared variable for drone position
drone_position = {"x": 0.0, "y": 0.0, "z": 0.0}
lock = threading.Lock()

# Callback function to process all received data but only update the drone's position
def receive_rigid_body_position(rigid_body_id, position, rotation):
    global drone_position
    with lock:
        drone_position["x"], drone_position["y"], drone_position["z"] = position

# Thread 1: Receives and processes all NatNet data
def receive_position():
    client = NatNetClient()
    client.rigid_body_listener = receive_rigid_body_position  # Callback for drone position
    client.run()  # Start listening for Motive data (receives all data)

# Thread 2: Prints only the drone’s updated position every 0.1 sec
def display_position():
    last_position = {"x": 0.0, "y": 0.0, "z": 0.0}  # Track last printed position
    while True:
        with lock:
            pos = drone_position.copy()  # Safe copy of the latest position

        # Only print if the position has changed
        if pos != last_position:
            print(f"Drone Position: x={pos['x']:.3f}, y={pos['y']:.3f}, z={pos['z']:.3f}")
            last_position = pos  # Update last printed position

        time.sleep(0.1)  # Print every 0.1 seconds if changed

# Create and start threads
thread1 = threading.Thread(target=receive_position, daemon=True)
thread2 = threading.Thread(target=display_position, daemon=True)

thread1.start()
thread2.start()

# Keep the script running
while True:
    time.sleep(1)
