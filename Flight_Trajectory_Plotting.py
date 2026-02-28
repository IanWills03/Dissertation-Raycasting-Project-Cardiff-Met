import threading
import time
import socket
import sys
from collections import deque
from MotionCapture_Files.NatNetClient import NatNetClient
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

drone_position = {"x": 0.0, "y": 0.0, "z": 0.0}
trajectory = []
lock = threading.Lock()
emergency_land_event = threading.Event()
script_exit_event = threading.Event()

# Matplotlib figure and axes objects - declared globally
fig = None
ax = None

emergency_entry_point = None
emergency_escape_path = []


forbidden_polygon = [
    (0.2, 1.3),
    (1.2, 1.3),
    (1.2, 0.3),
    (0.2, 0.3)
]

TELLO_IP = '192.168.10.1'
TELLO_PORT = 8889
BUFFER_SIZE = 1024
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
tello_address = (TELLO_IP, TELLO_PORT)


def point_in_polygon(x, y, polygon):
    num = len(polygon)
    j = num - 1
    inside = False
    for i in range(num):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and \
                (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-10) + xi):
            inside = not inside
        j = i
    return inside


def send_command(command, timeout=2.0):
    if emergency_land_event.is_set() and command not in ["back 60", "back 100", "land", "reboot", "stop"]:
        print(f"️ Emergency in progress. Skipping non-emergency command: '{command}'")
        return None
    # Check if the socket is still open before attempting to send data
    if sock._closed:
        print(f"️ Socket is closed. Cannot send command: '{command}'")
        return None

    try:
        print(f"Sending command: '{command}'")
        sock.sendto(command.encode('utf-8'), tello_address)
        sock.settimeout(timeout)
        msg, _ = sock.recvfrom(BUFFER_SIZE)
        response = msg.decode('utf-8').strip()
        print(f"Response: {response}")
        return response
    except socket.timeout:
        print(f"No response from Tello for command: '{command}' (timeout: {timeout}s)")
        return None
    except Exception as e:
        print(f" Command failed for '{command}' | Error: {e}")
        return None

def receive_rigid_body_position(rigid_body_id, position, rotation):
    global drone_position
    # Ensure you're tracking the correct rigid body ID from Motive
    # Change '1' to your drone's actual rigid body ID if different
    if rigid_body_id == 1:
        with lock:
            drone_position["x"] = position[0]
            drone_position["y"] = position[1]
            drone_position["z"] = position[2]


def receive_position():
    print("Starting NatNet client to receive position data...")
    client = NatNetClient()
    client.rigid_body_listener = receive_rigid_body_position
    try:
        client.run()
    except Exception as e:
        print(f"Error in NatNet client thread: {e}")
        script_exit_event.set()

def plot_trajectory():
    global trajectory, fig, ax, emergency_entry_point, emergency_escape_path  # Declare global usage
    plt.ion()  # Turn on interactive plotting mode
    fig, ax = plt.subplots(figsize=(8, 8))

    forbidden_patch = patches.Polygon(forbidden_polygon, closed=True, edgecolor='red', facecolor='none', linewidth=2,
                                      label="Forbidden Zone")
    ax.add_patch(forbidden_patch)

    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Z (meters)")
    ax.set_title("Live Drone Trajectory (Top-Down View)")

    # Original drone path (blue)
    drone_path_line, = ax.plot([], [], 'bo-', label="Drone Path (Normal)", alpha=0.7, markersize=5)

    escape_path_line, = ax.plot([], [], 'go-', label="Emergency Escape Path", alpha=0.8, markersize=5,
                                linewidth=2)  # 'g' for green

    entry_point_marker, = ax.plot([], [], 'r*', markersize=15,
                                  label="Entry Point to Forbidden Zone")  # 'r*' for red star

    ax.legend()
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(1.5, -1.5)  # Inverted Z-axis (plot Y-axis) for physical room orientation
    window_size = 10
    smoothed_x_queue = deque(maxlen=window_size)
    smoothed_z_queue = deque(maxlen=window_size)

    while not script_exit_event.is_set():  # Plot until script is told to exit
        with lock:
            x = drone_position["x"]
            z = drone_position["z"]

        current_pos = (x, z)

        if emergency_entry_point is None:
            trajectory.append(current_pos)

        smoothed_x_queue.append(x)
        smoothed_z_queue.append(z)

        # Calculate smoothed points for plotting
        if len(smoothed_x_queue) == window_size:
            avg_x = np.mean(list(smoothed_x_queue))
            avg_z = np.mean(list(smoothed_z_queue))

            if emergency_entry_point is None and trajectory:
                # Update the last point with smoothed values
                trajectory[-1] = (avg_x, avg_z)
            elif emergency_escape_path:
                emergency_escape_path[-1] = (avg_x, avg_z)

        x_vals_normal, z_vals_normal = zip(*trajectory) if trajectory else ([], [])
        drone_path_line.set_data(list(x_vals_normal), list(z_vals_normal))

        if emergency_escape_path:
            x_vals_escape, z_vals_escape = zip(*emergency_escape_path)
            escape_path_line.set_data(list(x_vals_escape), list(z_vals_escape))

        if emergency_entry_point:
            entry_point_marker.set_data([emergency_entry_point[0]], [emergency_entry_point[1]])

        all_x_coords = [p[0] for p in trajectory + emergency_escape_path if p is not None]
        all_z_coords = [p[1] for p in trajectory + emergency_escape_path if p is not None]

        if all_x_coords:
            data_min_x, data_max_x = min(all_x_coords), max(all_x_coords)
            data_min_z, data_max_z = min(all_z_coords), max(all_z_coords)

            padding = 0.5
            new_min_x = data_min_x - padding
            new_max_x = data_max_x + padding
            new_min_z = data_min_z - padding
            new_max_z = data_max_z + padding

            # Calculate center and half-range for both axes
            center_x = (new_min_x + new_max_x) / 2
            center_z = (new_min_z + new_max_z) / 2
            range_x = new_max_x - new_min_x
            range_z = new_max_z - new_min_z
            max_range = max(range_x, range_z)

            ax.set_xlim(center_x - max_range / 2, center_x + max_range / 2)
            ax.set_ylim(center_z + max_range / 2, center_z - max_range / 2)

        try:
            fig.canvas.draw()
            fig.canvas.flush_events()
        except Exception as e:
            print(f"Plotting update error (likely GUI closed): {e}")
            break

        time.sleep(0.1)

def monitor_and_react():
    global emergency_entry_point, emergency_escape_path
    has_landed = False
    while not has_landed and not script_exit_event.is_set():
        with lock:
            x = drone_position["x"]
            z = drone_position["z"]
            current_altitude = drone_position["y"]

        if point_in_polygon(x, z, forbidden_polygon):
            print("🚨 Drone in forbidden region! Initiating emergency actions.")
            emergency_land_event.set()

            if emergency_entry_point is None:
                emergency_entry_point = (x, z)
            print("Sending 'stop' command to halt current movement.")
            send_command("stop", timeout=2.0)
            time.sleep(1)

            with lock:
                x_after_stop = drone_position["x"]
                z_after_stop = drone_position["z"]


            if emergency_land_event.is_set():
                emergency_escape_path.append((x_after_stop, z_after_stop))

            if point_in_polygon(x_after_stop, z_after_stop, forbidden_polygon):
                print(f"️ Still in forbidden region after 'stop' attempt ({x_after_stop:.2f}, {z_after_stop:.2f}).")
                print("Attempting to move drone back 100cm (or multiple back commands)...")
                send_command("back 100", timeout=7.0)
                time.sleep(4)

                with lock:
                    x_final_check = drone_position["x"]
                    z_final_check = drone_position["z"]
                if emergency_land_event.is_set():
                    emergency_escape_path.append((x_final_check, z_final_check))

                if point_in_polygon(x_final_check, z_final_check, forbidden_polygon):
                    print(
                        f"️ STILL in forbidden region after retreat attempt ({x_final_check:.2f}, {z_final_check:.2f}). Landing directly in zone.")
                else:
                    print(
                        f"✅ Successfully moved out of forbidden region ({x_final_check:.2f}, {z_final_check:.2f}). Landing now.")
            else:
                print(
                    f"✅ Successfully stopped and out of forbidden region ({x_after_stop:.2f}, {z_after_stop:.2f}). Landing now.")

            send_command("land", timeout=10.0)  # Command to land
            time.sleep(5)

            has_landed = True
            script_exit_event.set()
            break

        elif emergency_land_event.is_set() and emergency_entry_point is not None:
            with lock:
                emergency_escape_path.append((x, z))

        time.sleep(0.1)


def patrolling():
    print("Patrolling the room...")
    command_execution_delay = 1.0
    command_timeout = 5.0

    for i in range(5):
        if emergency_land_event.is_set():
            print("Patrol aborted due to emergency (before next command).")
            break

        print(f"🌀 Patrol iteration {i + 1}")

        send_command("forward 250", timeout=command_timeout)
        if emergency_land_event.wait(command_execution_delay):  # Wait, but break if event set
            print("Patrol interrupted during forward 250 execution.")
            break

        send_command("ccw 90", timeout=command_timeout)
        if emergency_land_event.wait(command_execution_delay):
            print("Patrol interrupted during ccw 90 execution.")
            break

        send_command("forward 60", timeout=command_timeout)
        if emergency_land_event.wait(command_execution_delay):
            print("Patrol interrupted during forward 60 execution.")
            break

        send_command("ccw 90", timeout=command_timeout)
        if emergency_land_event.wait(command_execution_delay):
            print("Patrol interrupted during ccw 90 execution.")
            break

        send_command("forward 250", timeout=command_timeout)
        if emergency_land_event.wait(command_execution_delay):
            print("Patrol interrupted during forward 250 execution.")
            break

        send_command("cw 90", timeout=command_timeout)
        if emergency_land_event.wait(command_execution_delay):
            print("Patrol interrupted during cw 90 execution.")
            break

        send_command("forward 60", timeout=command_timeout)
        if emergency_land_event.wait(command_execution_delay):
            print("Patrol interrupted during forward 60 execution.")
            break

        send_command("cw 90", timeout=command_timeout)
        if emergency_land_event.wait(command_execution_delay):
            print("Patrol interrupted during cw 90 execution.")
            break

    if not emergency_land_event.is_set():
        print("Patrolling sequence completed.")


def patrol_thread_func():
    print("Patrol thread starting (waiting for takeoff and initial flight)...")
    time.sleep(5)  # Wait a bit after takeoff before starting patrol
    patrolling()
    print("Patrol thread finished.")

def tello_startup():
    print("Initializing Tello...")
    # Increase timeout for initial commands as Tello can be slow to respond
    if send_command("command", timeout=5.0) == "ok":
        send_command("battery?", timeout=2.0)  # Battery command is usually fast
        # Increase takeoff timeout significantly as it's a critical and sometimes slow command
        takeoff_response = send_command("takeoff", timeout=10.0)
        if takeoff_response == "ok":
            send_command("up 30", timeout=5.0)  # Ascend to 30 cm after takeoff
            print("Tello is now flying.")
            return True  # Indicate successful startup
        else:
            print("Tello takeoff failed. Response:", takeoff_response)
            script_exit_event.set()  # Signal exit if takeoff fails
            return False
    else:
        print("Failed to enter Tello command mode.")
        script_exit_event.set()  # Signal exit if command mode fails
        return False

if __name__ == "__main__":
    print("Starting drone control script...")

    # Define threads (plot_thread is non-daemon for final plot display)
    natnet_thread = threading.Thread(target=receive_position, daemon=True)
    plot_thread = threading.Thread(target=plot_trajectory)  # Make non-daemon to keep plot window open
    monitor_thread = threading.Thread(target=monitor_and_react)
    patrol_thread = threading.Thread(target=patrol_thread_func)

    # Start essential threads
    natnet_thread.start()
    plot_thread.start()
    monitor_thread.start()

    print("Waiting for motion capture system to establish initial position...")
    time.sleep(5)


    with lock:
        x_start = drone_position["x"]
        z_start = drone_position["z"]

    if point_in_polygon(x_start, z_start, forbidden_polygon):
        print(
            f"️ Drone is inside forbidden region at startup ({x_start:.2f}, {z_start:.2f}). Aborting mission and attempting to land.")
        send_command("land", timeout=10.0)  # Attempt to land immediately
        time.sleep(5)  # Give it time to land
        script_exit_event.set()  # Signal all threads to exit
    else:
        print(f"Drone starting position ({x_start:.2f}, {z_start:.2f}) is safe. Proceeding.")
        if tello_startup():
            patrol_thread.start()
        else:
            print("Tello startup failed, not starting patrol.")
            script_exit_event.set()

    try:
        while not script_exit_event.is_set():
            time.sleep(1)
        print("Script exit event set. Initiating final cleanup.")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Signalling emergency land and cleanup.")
        emergency_land_event.set()  # Set emergency to stop patrolling
        send_command("land", timeout=10.0)  # Attempt to land on manual interrupt
        time.sleep(5)  # Give it time to land
        script_exit_event.set()  # Signal all threads to wrap up

    finally:
        print("Attempting to join threads for graceful shutdown...")
        # Join non-daemon threads to allow them to complete their final tasks
        # Use a timeout to prevent indefinite blocking if a thread hangs

        # Monitor thread should have already exited if emergency_land_event was set
        if monitor_thread.is_alive():
            print("Joining monitor_thread...")
            monitor_thread.join(timeout=10)
        if patrol_thread.is_alive():
            print("Joining patrol_thread...")
            patrol_thread.join(timeout=10)
        print("All active threads should be terminated or joined.")
        if not sock._closed:
            print("Closing Tello UDP socket.")
            sock.close()
        else:
            print("Tello UDP socket was already closed.")

        plt.ioff()
        # Check if fig exists and the figure number is valid before calling plt.show()
        if fig is not None and plt.fignum_exists(fig.number):
            print("Displaying final trajectory plot. Close window to exit.")
            plt.show(block=True)
        else:
            print("Plot figure not found or already closed.")

    print("Script execution fully complete. Goodbye!")
    sys.exit(0)