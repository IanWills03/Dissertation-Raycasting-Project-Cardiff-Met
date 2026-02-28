import threading
import time
import socket
import sys
from collections import deque
from MotionCapture_Files.NatNetClient import NatNetClient
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math

# ---------------------------
# Global Variables
# These are declared globally so they can be accessed and modified by all threads
drone_position = {"x": 0.0, "y": 0.0, "z": 0.0}
trajectory = []  # Stores flight path points for plotting
lock = threading.Lock()  # Protects access to shared variables
emergency_land_event = threading.Event()  # Signals an emergency landing
script_exit_event = threading.Event()  # Signals graceful script termination
retreated_to_safe_distance = threading.Event()  # New event to signal safe retreat after emergency

# Matplotlib figure and axes objects - declared globally for access across threads
fig = None
ax = None
position_text = None  # Global variable to hold the Matplotlib text object for drone position
emergency_status_text = None  # Global variable for emergency status display

# Global variables for plotting emergency events and desired path
emergency_entry_point = None  # (x, z) where drone first entered forbidden zone
emergency_escape_path = []  # Trajectory points during emergency escape
desired_trajectory_points = []  # Pre-calculated ideal path

# Global variables for ray casting visualization
ray_start_plot = None
ray_end_plot = None
intersection_points_plot = []

# Forbidden polygon zone (in meters)
forbidden_polygon = [
    (0.3, 0.3),  # bottom-left (X_min, Z_min)
    (1.3, 0.3),  # bottom-right (X_max, Z_min)
    (1.3, 1.3),  # top-right (X_max, Z_max)
    (0.3, 1.3)  # top-left (X_min, Z_max)
]

# Warning margin around the forbidden polygon (in meters)
WARNING_MARGIN = 0.5  # Drone should react if it comes within this distance of the forbidden zone

# Tello UDP Setup
TELLO_IP = '192.168.10.1'
TELLO_PORT = 8889
BUFFER_SIZE = 1024
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
tello_address = (TELLO_IP, TELLO_PORT)


# ---------------------------
# Helper Functions

def point_in_polygon(x, y, polygon):
    """
    Checks if a point (x, y) is inside a given polygon using the ray-casting algorithm.
    Returns:
        - inside (bool): True if point is inside, False otherwise.
        - ray_start (tuple): (x, y) of the ray's origin.
        - ray_end (tuple): (x_far, y) of the ray's end point.
        - intersections (list): List of (x, y) tuples for intersection points.
    """
    num = len(polygon)
    j = num - 1
    inside = False
    intersections = []

    # Define the ray: from (x, y) extending horizontally to a point far to the right
    x_far = 10.0  # A fixed point far to the right for the ray's end
    ray_start = (x, y)
    ray_end = (x_far, y)

    for i in range(num):
        p1 = polygon[i]
        p2 = polygon[j]

        # Check for intersection between the horizontal ray and the polygon segment (p1, p2)
        if ((p1[1] > y) != (p2[1] > y)):  # If the segment crosses the ray's y-level
            # Calculate x-coordinate of intersection with the horizontal line y
            x_intersect = (p2[0] - p1[0]) * (y - p1[1]) / ((p2[1] - p1[1]) + 1e-10) + p1[0]

            # Check if the intersection point is to the right of the ray's origin (drone's X)
            # and within the x-range of the segment itself.
            if x_intersect >= x and (min(p1[0], p2[0]) <= x_intersect <= max(p1[0], p2[0])):
                inside = not inside
                intersections.append((x_intersect, y))
        j = i
    return inside, ray_start, ray_end, intersections


def is_within_warning_zone(x, z, polygon, margin):
    """
    Checks if a point (x, z) is within a certain margin of the forbidden polygon's bounding box.
    This creates a simplified 'warning zone' around the forbidden area.
    """
    min_x_poly = min(p[0] for p in polygon)
    max_x_poly = max(p[0] for p in polygon)
    min_z_poly = min(p[1] for p in polygon)
    max_z_poly = max(p[1] for p in polygon)

    # Expand the polygon's bounding box by the margin
    expanded_min_x = min_x_poly - margin
    expanded_max_x = max_x_poly + margin
    expanded_min_z = min_z_poly - margin
    expanded_max_z = max_z_poly + margin

    # Check if the point is within this expanded bounding box
    return expanded_min_x <= x <= expanded_max_x and \
        expanded_min_z <= z <= expanded_max_z


def send_command(command, timeout=2.0):
    """
    Sends a command to the Tello drone and waits for a response.
    Includes a check to skip non-emergency commands during an emergency.
    """
    # If an emergency is active, only allow specific emergency-related commands
    if emergency_land_event.is_set() and command not in ["back 60", "back 100", "back 150", "back 200", "land", "stop",
                                                         "reboot", "back 30"]:
        print(f"⚠️ Emergency in progress. Skipping non-emergency command: '{command}'")
        return None

    # Ensure socket is open before sending
    if sock._closed:
        print(f"⚠️ Socket is closed. Cannot send command: '{command}'")
        return None

    try:
        print(f"Sending command: '{command}'")
        sock.sendto(command.encode('utf-8'), tello_address)
        sock.settimeout(timeout)  # Set a timeout for receiving response
        msg, _ = sock.recvfrom(BUFFER_SIZE)
        response = msg.decode('utf-8').strip()
        print(f"Response: {response}")
        return response
    except socket.timeout:
        print(f"No response from Tello for command: '{command}' (timeout: {timeout}s)")
        return None
    except Exception as e:
        print(f"⚠️ Command failed for '{command}' | Error: {e}")
        return None


# ---------------------------
# NatNet (Motion Capture) Thread

def receive_rigid_body_position(rigid_body_id, position, rotation):
    """
    Callback function to receive drone's position from NatNet.
    Updates the shared drone_position dictionary.
    """
    global drone_position
    if rigid_body_id == 1:  # Assuming your drone's rigid body ID in Motive is 1
        with lock:
            drone_position["x"] = position[0]
            drone_position["y"] = position[1]  # OptiTrack Y is typically vertical (altitude)
            drone_position["z"] = position[2]


def receive_position():
    """
    Thread function to run the NatNet client and receive position data.
    """
    print("Starting NatNet client to receive position data...")
    client = NatNetClient()
    client.rigid_body_listener = receive_rigid_body_position
    try:
        client.run()
    except Exception as e:
        print(f"Error in NatNet client thread: {e}")
        script_exit_event.set()


# ---------------------------
# Plotting Thread (Live Plot)

def set_emergency_status_text(status_text):
    """
    Updates the emergency status text on the plot.
    """
    global emergency_status_text
    if emergency_status_text:
        emergency_status_text.set_text(status_text)
        # Set color based on status
        if "EMERGENCY" in status_text:
            emergency_status_text.set_color('red')
        else:
            emergency_status_text.set_color('blue')


def plot_trajectory():
    global trajectory, fig, ax, emergency_entry_point, emergency_escape_path, desired_trajectory_points, position_text, emergency_status_text, ray_start_plot, ray_end_plot, intersection_points_plot, retreated_to_safe_distance

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))

    # Add forbidden zone patch
    forbidden_patch = patches.Polygon(forbidden_polygon, closed=True, edgecolor='red', facecolor='red', alpha=0.2,
                                      linewidth=2,
                                      label="Forbidden Zone")
    ax.add_patch(forbidden_patch)

    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Z (meters)")
    ax.set_title("Live Drone Trajectory (Top-Down View)")

    desired_path_line, = ax.plot([], [], 'k--', label="Desired Path", alpha=0.7,
                                 linewidth=2)  # Slightly thicker desired path
    drone_path_line, = ax.plot([], [], 'b-', label="Actual Path (Normal)", alpha=0.8, markersize=5,
                               linewidth=3)  # Thicker blue line
    escape_path_line, = ax.plot([], [], 'g-', label="Emergency Escape Path", alpha=0.9, markersize=7,
                                linewidth=4)  # Very thick green line
    entry_point_marker, = ax.plot([], [], 'rX', markersize=20,
                                  label="Entry Point to Forbidden Zone")  # Larger 'X' marker

    ray_line, = ax.plot([], [], 'c:', label="Ray Cast", alpha=0.5, linewidth=1)  # Cyan dashed line for ray
    intersection_markers, = ax.plot([], [], 'ro', markersize=8,
                                    label="Ray Intersections")  # Red circles for intersections

    ax.legend()
    ax.set_aspect('equal', adjustable='box')  # Maintain aspect ratio

    min_x_poly = min(p[0] for p in forbidden_polygon)
    max_x_poly = max(p[0] for p in forbidden_polygon)
    min_z_poly = min(p[1] for p in forbidden_polygon)
    max_z_poly = max(p[1] for p in forbidden_polygon)

    padding_initial = 0.5  # Add some initial padding
    ax.set_xlim(min_x_poly - padding_initial, max_x_poly + padding_initial)
    ax.set_ylim(max_z_poly + padding_initial, min_z_poly - padding_initial)  # Inverted Y-axis for Z-coordinate

    position_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                            fontsize=10, verticalalignment='top',
                            bbox=dict(boxstyle="round,pad=0.3", fc="cyan", ec="b", lw=1, alpha=0.6))

    emergency_status_text = ax.text(0.02, 0.93, 'Status: Normal', transform=ax.transAxes,
                                    fontsize=10, verticalalignment='top', color='blue',
                                    bbox=dict(boxstyle="round,pad=0.3", fc="yellow", ec="darkblue", lw=1, alpha=0.6))

    # For trajectory smoothing
    window_size = 20  # Number of points for moving average.
    smoothed_x_queue = deque(maxlen=window_size)
    smoothed_z_queue = deque(maxlen=window_size)

    while not script_exit_event.is_set():
        with lock:
            x = drone_position["x"]
            z = drone_position["z"]
            altitude = drone_position["y"]  # Get altitude (OptiTrack Y) for display
            is_emergency = emergency_land_event.is_set()
            has_retreated_safely = retreated_to_safe_distance.is_set()

            # Get ray casting data from global variables
            current_ray_start = ray_start_plot
            current_ray_end = ray_end_plot
            current_intersections = intersection_points_plot

        current_pos = (x, z)

        # Update the position text on the plot
        if position_text:
            position_text.set_text(f'Pos: X={x:.2f}m, Z={z:.2f}m, Y(Alt)={altitude:.2f}m')

        # Update emergency status text with ray casting info
        status_msg = "Status: Normal"
        if is_emergency and not has_retreated_safely:
            status_msg = "EMERGENCY: EVADING!"
        elif has_retreated_safely:
            status_msg = "EMERGENCY: RETREATED SAFELY!"

        # Add ray casting status if data is available
        if current_ray_start is not None:
            num_intersections = len(current_intersections)
            if num_intersections % 2 == 1:
                status_msg += f" (Inside: {num_intersections} intersections)"
            else:
                status_msg += f" (Outside: {num_intersections} intersections)"

        set_emergency_status_text(status_msg)

        # Add current raw position to queues for smoothing
        smoothed_x_queue.append(x)
        smoothed_z_queue.append(z)

        # Append to trajectory lists based on emergency status
        if not is_emergency and not has_retreated_safely:  # If not in emergency and not just finished emergency
            trajectory.append(current_pos)
            # Ensure emergency plotting elements are cleared if we're back to normal operation
            if emergency_escape_path or emergency_entry_point:
                emergency_escape_path.clear()
                emergency_entry_point = None
        elif is_emergency:  # If in emergency, add to emergency escape path
            emergency_escape_path.append(current_pos)
        # If emergency just ended (retreated_to_safe_distance is set), we stop adding to escape path
        # The monitor thread will then signal script_exit_event to land

        # Apply smoothing when enough data points are available
        if len(smoothed_x_queue) == window_size:
            avg_x = np.mean(list(smoothed_x_queue))
            avg_z = np.mean(list(smoothed_z_queue))

            # Update the last point in the *active* trajectory list with the smoothed value
            if not is_emergency and not has_retreated_safely and trajectory:
                trajectory[-1] = (avg_x, avg_z)
            elif is_emergency and emergency_escape_path:
                emergency_escape_path[-1] = (avg_x, avg_z)

        # Update plot data based on the current state
        x_vals_normal, z_vals_normal = zip(*trajectory) if trajectory else ([], [])
        drone_path_line.set_data(list(x_vals_normal), list(z_vals_normal))

        if emergency_escape_path:
            x_vals_escape, z_vals_escape = zip(*emergency_escape_path)
            escape_path_line.set_data(list(x_vals_escape), list(z_vals_escape))
        else:
            # Clear escape path line if not in emergency
            escape_path_line.set_data([], [])

        if desired_trajectory_points:
            desired_x_vals, desired_z_vals = zip(*desired_trajectory_points)
            desired_path_line.set_data(list(desired_x_vals), list(desired_z_vals))

        if emergency_entry_point:
            entry_point_marker.set_data([emergency_entry_point[0]], [emergency_entry_point[1]])
        else:
            # Clear entry point marker if not in emergency
            entry_point_marker.set_data([], [])

        # Update ray casting plot elements
        if current_ray_start and current_ray_end:
            ray_line.set_data([current_ray_start[0], current_ray_end[0]], [current_ray_start[1], current_ray_end[1]])
        else:
            ray_line.set_data([], [])  # Clear ray if no data

        if current_intersections:
            intersect_x = [p[0] for p in current_intersections]
            intersect_z = [p[1] for p in current_intersections]
            intersection_markers.set_data(intersect_x, intersect_z)
        else:
            intersection_markers.set_data([], [])  # Clear markers if no intersections

        # Dynamic plot limits to auto-zoom on trajectory
        all_x_coords = [p[0] for p in trajectory + emergency_escape_path + desired_trajectory_points if p is not None]
        all_z_coords = [p[1] for p in trajectory + emergency_escape_path + desired_trajectory_points if p is not None]

        # Include ray and intersection points for plot limits
        if current_ray_start:
            all_x_coords.append(current_ray_start[0])
            all_z_coords.append(current_ray_start[1])
            all_x_coords.append(current_ray_end[0])
            all_z_coords.append(current_ray_end[1])
        for p in current_intersections:
            all_x_coords.append(p[0])
            all_z_coords.append(p[1])

        if all_x_coords:
            data_min_x, data_max_x = min(all_x_coords), max(all_x_coords)
            data_min_z, data_max_z = min(all_z_coords), max(all_z_coords)

            # Include forbidden polygon boundaries in min/max for consistent scaling
            all_x_coords.extend([p[0] for p in forbidden_polygon])
            all_z_coords.extend([p[1] for p in forbidden_polygon])

            data_min_x, data_max_x = min(all_x_coords), max(all_x_coords)
            data_min_z, data_max_z = min(all_z_coords), max(all_z_coords)

            padding = 0.5  # Add 0.5m padding around the data
            new_min_x = data_min_x - padding
            new_max_x = data_max_x + padding
            new_min_z = data_min_z - padding
            new_max_z = data_max_z + padding

            # Calculate square limits to maintain aspect ratio
            center_x = (new_min_x + new_max_x) / 2
            center_z = (new_min_z + new_max_z) / 2
            range_x = new_max_x - new_min_x
            range_z = new_max_z - new_min_z
            max_range = max(range_x, range_z)

            # Adjust limits to ensure square aspect ratio and padding
            ax.set_xlim(center_x - max_range / 2, center_x + max_range / 2)
            ax.set_ylim(center_z + max_range / 2, center_z - max_range / 2)  # Still inverted Z-axis

        try:
            plt.pause(0.001)  # Small pause to allow plot to update
        except Exception as e:
            print(f"Plotting update error (likely GUI closed): {e}")
            break

    if fig is not None and plt.fignum_exists(fig.number):
        plt.close(fig)
        print("Plot window closed by script.")


# ---------------------------
# Monitor and React Thread (with robust retreat)

def monitor_and_react():
    """
    Monitors drone position from OptiTrack and initiates emergency landing/retreat
    if it enters the forbidden region or the warning zone.
    """
    global emergency_entry_point, emergency_escape_path, ray_start_plot, ray_end_plot, intersection_points_plot, retreated_to_safe_distance
    has_landed = False

    # This loop will only start after it's explicitly started in the main thread,
    # which is now after Tello takeoff and initial OptiTrack data is flowing.
    while not has_landed and not script_exit_event.is_set():
        with lock:
            x = drone_position["x"]
            z = drone_position["z"]

        # Call point_in_polygon and get ray casting data for plotting
        is_in_forbidden_zone, current_ray_start, current_ray_end, current_intersections = point_in_polygon(x, z,
                                                                                                           forbidden_polygon)
        is_in_warning_zone = is_within_warning_zone(x, z, forbidden_polygon, WARNING_MARGIN)

        with lock:  # Update global plotting variables, protected by lock
            ray_start_plot = current_ray_start
            ray_end_plot = current_ray_end
            intersection_points_plot = current_intersections

            # Trigger emergency if in forbidden zone or warning zone, AND emergency not already active
            if (is_in_forbidden_zone or is_in_warning_zone) and not emergency_land_event.is_set():
                emergency_land_event.set()
                print(
                    f"🚨 EMERGENCY TRIGGERED at ({x:.2f}, {z:.2f})! In forbidden: {is_in_forbidden_zone}, In warning: {is_in_warning_zone}")
            # If drone has retreated safely, clear the emergency event
            # This logic will be handled by the monitor thread completing its retreat and landing sequence
            # and setting script_exit_event.

        # Only react if an emergency is active AND we haven't already landed
        if emergency_land_event.is_set() and not has_landed:
            print("🚨 Drone in forbidden region or warning zone! Initiating emergency actions.")

            if emergency_entry_point is None:  # Capture first entry point of the emergency
                emergency_entry_point = (x, z)
                emergency_escape_path.clear()  # Clear previous escape path for a new emergency

            # --- IMMEDIATE CANCELLATION ---
            print("Sending 'stop' command to halt current movement.")
            send_command("stop", timeout=2.0)
            time.sleep(1)  # Give drone time to stop and stabilize

            # --- Decisive Retreat (back 200cm) ---
            print("Executing decisive retreat: back 200cm to clearly avoid the forbidden zone.")
            send_command("back 200", timeout=7.0)  # Send a large retreat command
            time.sleep(4)  # Give more time for drone to execute the command

            with lock:  # Re-check position after the decisive retreat
                current_x_after_retreat = drone_position["x"]
                current_z_after_retreat = drone_position["z"]

            # Check if drone is now outside the warning zone after retreat
            is_out_of_warning_zone = not is_within_warning_zone(current_x_after_retreat, current_z_after_retreat,
                                                                forbidden_polygon, WARNING_MARGIN)

            if is_out_of_warning_zone:
                print(
                    f"✅ Drone successfully moved out of warning zone after decisive retreat ({current_x_after_retreat:.2f}, {current_z_after_retreat:.2f}).")
                retreated_to_safe_distance.set()  # Signal that drone has retreated safely
            else:
                print(
                    f"⚠️ Despite decisive retreat, still in warning zone or forbidden region ({current_x_after_retreat:.2f}, {current_z_after_retreat:.2f}). This indicates a significant issue or very rapid drone movement.")
                # Even if still in warning zone, we proceed to land as this is the final action.

            # --- Final Landing ---
            print("Proceeding to land.")
            send_command("land", timeout=10.0)
            time.sleep(5)  # Give time for landing
            has_landed = True
            script_exit_event.set()  # Signal other threads to exit
            break  # Exit monitor_and_react loop

        time.sleep(0.1)  # Check position every 100ms


# ---------------------------
# Desired Trajectory Calculation Function

def calculate_desired_trajectory(start_x, start_z, initial_heading_degrees=0):
    """
    Calculates the ideal flight path based on the patrol commands for plotting.
    This is a simulation of the intended path, not a real-time control algorithm.
    """
    global desired_trajectory_points
    desired_trajectory_points.clear()  # Clear previous desired path for recalculation

    current_x = start_x
    current_z = start_z
    current_heading_rad = math.radians(initial_heading_degrees)

    desired_trajectory_points.append((current_x, current_z))

    segment_size_cm = 50  # For breaking down longer movements in simulation

    for i in range(5):  # Number of patrol iterations
        # forward 280 (matches patrolling function)
        total_dist_cm = 280
        moved_cm = 0
        while moved_cm < total_dist_cm:
            dist_to_move_cm = min(segment_size_cm, total_dist_cm - moved_cm)
            dist_to_move_m = dist_to_move_cm / 100.0

            # Calculate movement based on current heading
            delta_x = dist_to_move_m * math.sin(current_heading_rad)
            delta_z = dist_to_move_m * math.cos(current_heading_rad)

            current_x += delta_x
            current_z += delta_z
            desired_trajectory_points.append((current_x, current_z))
            moved_cm += dist_to_move_cm

        # ccw 90
        current_heading_rad += math.radians(90)
        desired_trajectory_points.append((current_x, current_z))

        # forward 50 (matches patrolling function)
        total_dist_cm = 50
        moved_cm = 0
        while moved_cm < total_dist_cm:
            dist_to_move_cm = min(segment_size_cm, total_dist_cm - moved_cm)
            dist_to_move_m = dist_to_move_cm / 100.0

            delta_x = dist_to_move_m * math.sin(current_heading_rad)
            delta_z = dist_to_move_m * math.cos(current_heading_rad)

            current_x += delta_x
            current_z += delta_z
            desired_trajectory_points.append((current_x, current_z))
            moved_cm += dist_to_move_cm

        # ccw 90
        current_heading_rad += math.radians(90)
        desired_trajectory_points.append((current_x, current_z))

        # forward 280 (matches patrolling function)
        total_dist_cm = 280
        moved_cm = 0
        while moved_cm < total_dist_cm:
            dist_to_move_cm = min(segment_size_cm, total_dist_cm - moved_cm)
            dist_to_move_m = dist_to_move_cm / 100.0

            delta_x = dist_to_move_m * math.sin(current_heading_rad)
            delta_z = dist_to_move_m * math.cos(current_heading_rad)

            current_x += delta_x
            current_z += delta_z
            desired_trajectory_points.append((current_x, current_z))
            moved_cm += dist_to_move_cm

        # cw 90
        current_heading_rad -= math.radians(90)
        desired_trajectory_points.append((current_x, current_z))

        # forward 50 (matches patrolling function)
        total_dist_cm = 50
        moved_cm = 0
        while moved_cm < total_dist_cm:
            dist_to_move_cm = min(segment_size_cm, total_dist_cm - moved_cm)
            dist_to_move_m = dist_to_move_cm / 100.0

            delta_x = dist_to_move_m * math.sin(current_heading_rad)
            delta_z = dist_to_move_m * math.cos(current_heading_rad)

            current_x += delta_x
            current_z += delta_z
            desired_trajectory_points.append((current_x, current_z))
            moved_cm += dist_to_move_cm

        # cw 90
        current_heading_rad -= math.radians(90)
        desired_trajectory_points.append((current_x, current_z))

    print("Desired trajectory calculated.")


# ---------------------------
# Patrolling Thread (Tello commands)

def patrolling():
    """
    Sends a sequence of commands to the Tello drone to perform a patrol.
    Uses segmented movements for better responsiveness to emergency events.
    """
    print("Patrolling the room...")
    command_execution_delay = 1.0  # Time to wait after sending a command
    command_timeout = 5.0  # Timeout for Tello response
    segment_size_cm = 50  # cm, for breaking down longer movements

    for i in range(5):  # Number of patrol iterations
        # Check for emergency or script exit before starting a new patrol segment
        if emergency_land_event.is_set() or script_exit_event.is_set():
            print("Patrol aborted due to emergency or script exit.")
            break

        print(f"🌀 Patrol iteration {i + 1}")

        # forward 280
        total_forward_dist = 280
        current_moved = 0
        while current_moved < total_forward_dist and not emergency_land_event.is_set() and not script_exit_event.is_set():
            dist_to_move = min(segment_size_cm, total_forward_dist - current_moved)
            send_command(f"forward {dist_to_move}", timeout=command_timeout)
            if emergency_land_event.wait(command_execution_delay) or script_exit_event.is_set():
                print(f"Patrol interrupted during forward {dist_to_move} execution.")
                break
            current_moved += dist_to_move
        if emergency_land_event.is_set() or script_exit_event.is_set(): break

        send_command("ccw 90", timeout=command_timeout)
        if emergency_land_event.wait(command_execution_delay) or script_exit_event.is_set():
            print("Patrol interrupted during ccw 90 execution.")
            break

        # forward 50
        total_forward_dist = 50
        current_moved = 0
        while current_moved < total_forward_dist and not emergency_land_event.is_set() and not script_exit_event.is_set():
            dist_to_move = min(segment_size_cm, total_forward_dist - current_moved)
            send_command(f"forward {dist_to_move}", timeout=command_timeout)
            if emergency_land_event.wait(command_execution_delay) or script_exit_event.is_set():
                print(f"Patrol interrupted during forward {dist_to_move} execution.")
                break
            current_moved += dist_to_move
        if emergency_land_event.is_set() or script_exit_event.is_set(): break

        send_command("ccw 90", timeout=command_timeout)
        if emergency_land_event.wait(command_execution_delay) or script_exit_event.is_set():
            print("Patrol interrupted during ccw 90 execution.")
            break

        # forward 280
        total_forward_dist = 280
        current_moved = 0
        while current_moved < total_forward_dist and not emergency_land_event.is_set() and not script_exit_event.is_set():
            dist_to_move = min(segment_size_cm, total_forward_dist - current_moved)
            send_command(f"forward {dist_to_move}", timeout=command_timeout)
            if emergency_land_event.wait(command_execution_delay) or script_exit_event.is_set():
                print(f"Patrol interrupted during forward {dist_to_move} execution.")
                break
            current_moved += dist_to_move
        if emergency_land_event.is_set() or script_exit_event.is_set(): break

        send_command("cw 90", timeout=command_timeout)
        if emergency_land_event.wait(command_execution_delay) or script_exit_event.is_set():
            print("Patrol interrupted during cw 90 execution.")
            break

        # forward 50
        total_forward_dist = 50
        current_moved = 0
        while current_moved < total_forward_dist and not emergency_land_event.is_set() and not script_exit_event.is_set():
            dist_to_move = min(segment_size_cm, total_forward_dist - current_moved)
            send_command(f"forward {dist_to_move}", timeout=command_timeout)
            if emergency_land_event.wait(command_execution_delay) or script_exit_event.is_set():
                print(f"Patrol interrupted during forward {dist_to_move} execution.")
                break
            current_moved += dist_to_move
        if emergency_land_event.is_set() or script_exit_event.is_set(): break

        send_command("cw 90", timeout=command_timeout)
        if emergency_land_event.wait(command_execution_delay) or script_exit_event.is_set():
            print("Patrol interrupted during cw 90 execution.")
            break

    if not emergency_land_event.is_set() and not script_exit_event.is_set():
        print("Patrolling sequence completed.")


def patrol_thread_func():
    """Wrapper function for patrolling to be used as a thread target."""
    print("Patrol thread starting...")
    patrolling()
    print("Patrol thread finished.")


# ---------------------------
# Initialization & Startup

def tello_startup():
    """
    Initializes Tello command mode, checks battery, takes off, and ascends.
    """
    print("Initializing Tello...")
    if send_command("command", timeout=5.0) == "ok":
        send_command("battery?", timeout=2.0)
        takeoff_response = send_command("takeoff", timeout=10.0)
        if takeoff_response == "ok":
            send_command("up 30", timeout=5.0)  # Ascend to a safe height
            time.sleep(3)  # Give OptiTrack time to reflect the new position after ascending
            print("Tello is now flying and stabilized.")
            return True
        else:
            print("Tello takeoff failed. Response:", takeoff_response)
            script_exit_event.set()  # Signal exit if takeoff fails
            return False
    else:
        print("Failed to enter Tello command mode. Is Tello connected?")
        script_exit_event.set()
        return False


# ---------------------------
# Final Plotting Function (called at the end)

def final_plot_trajectory(final_trajectory, final_emergency_escape_path, final_desired_trajectory_points,
                          final_forbidden_polygon):
    """
    Creates a final, static plot of the drone's trajectory after the flight is complete.
    """
    print("Generating final trajectory plot...")
    fig_final, ax_final = plt.subplots(figsize=(10, 10))  # Larger figure for final plot

    # Add forbidden zone patch
    forbidden_patch_final = patches.Polygon(final_forbidden_polygon, closed=True, edgecolor='red', facecolor='red',
                                            alpha=0.2, linewidth=2,
                                            label="Forbidden Zone")
    ax_final.add_patch(forbidden_patch_final)

    ax_final.set_xlabel("X (meters)")
    ax_final.set_ylabel("Z (meters)")
    ax_final.set_title("Final Drone Trajectory (Top-Down View)")

    # Plot desired path
    if final_desired_trajectory_points:
        desired_x_vals, desired_z_vals = zip(*final_desired_trajectory_points)
        ax_final.plot(list(desired_x_vals), list(desired_z_vals), 'k--', label="Desired Path", alpha=0.7, linewidth=2)

    # Plot actual path (normal)
    if final_trajectory:
        x_vals_normal, z_vals_normal = zip(*final_trajectory)
        ax_final.plot(list(x_vals_normal), list(z_vals_normal), 'b-', label="Actual Path (Normal)", alpha=0.8,
                      markersize=5, linewidth=3)

    # Plot emergency escape path
    if final_emergency_escape_path:
        x_vals_escape, z_vals_escape = zip(*final_emergency_escape_path)
        ax_final.plot(list(x_vals_escape), list(z_vals_escape), 'g-', label="Emergency Escape Path", alpha=0.9,
                      markersize=7, linewidth=4)

        # Plot the entry point to the forbidden zone for the final plot
        if emergency_entry_point:  # Use the global entry point
            ax_final.plot(emergency_entry_point[0], emergency_entry_point[1], 'rX', markersize=20,
                          label="Entry Point to Forbidden Zone")

    ax_final.legend()
    ax_final.set_aspect('equal', adjustable='box')

    # Auto-scale limits based on all data
    all_x_coords = []
    all_z_coords = []
    if final_trajectory:
        all_x_coords.extend([p[0] for p in final_trajectory])
        all_z_coords.extend([p[1] for p in final_trajectory])
    if final_emergency_escape_path:
        all_x_coords.extend([p[0] for p in final_emergency_escape_path])
        all_z_coords.extend([p[1] for p in final_emergency_escape_path])
    if final_desired_trajectory_points:
        all_x_coords.extend([p[0] for p in final_desired_trajectory_points])
        all_z_coords.extend([p[1] for p in final_desired_trajectory_points])

    # Include forbidden polygon boundaries in min/max for consistent scaling
    all_x_coords.extend([p[0] for p in final_forbidden_polygon])
    all_z_coords.extend([p[1] for p in final_forbidden_polygon])

    if all_x_coords:
        data_min_x, data_max_x = min(all_x_coords), max(all_x_coords)
        data_min_z, data_max_z = min(all_z_coords), max(all_z_coords)

        padding = 0.5
        new_min_x = data_min_x - padding
        new_max_x = data_max_x + padding
        new_min_z = data_min_z - padding
        new_max_z = data_max_z + padding

        center_x = (new_min_x + new_max_x) / 2
        center_z = (new_min_z + new_max_z) / 2
        range_x = new_max_x - new_min_x
        range_z = new_max_z - new_min_z
        max_range = max(range_x, range_z)

        ax_final.set_xlim(center_x - max_range / 2, center_x + max_range / 2)
        ax_final.set_ylim(center_z + max_range / 2, center_z - max_range / 2)  # Inverted Y-axis for Z-coordinate

    plt.show(block=True)  # Block to keep the final plot open until closed manually
    print("Final plot displayed.")


# ---------------------------
# Main Execution Block

if __name__ == "__main__":
    print("Starting drone control script...")

    # --- IMPORTANT: Reset events at the start of every run ---
    emergency_land_event.clear()
    script_exit_event.clear()
    retreated_to_safe_distance.clear()  # Clear new event

    # Create and start threads
    natnet_thread = threading.Thread(target=receive_position, daemon=True)  # Daemon so it exits with main
    plot_thread = threading.Thread(target=plot_trajectory)  # Keep live plot for debugging
    # monitor_thread will be started later, after Tello startup
    monitor_thread = threading.Thread(target=monitor_and_react)
    patrol_thread = threading.Thread(target=patrol_thread_func)

    natnet_thread.start()
    plot_thread.start()  # Start the live plotting thread

    print("Waiting for motion capture system to establish initial position...")
    time.sleep(5)  # Give NatNet client time to connect and get initial data

    with lock:
        x_initial_system_start = drone_position["x"]
        z_initial_system_start = drone_position["z"]

    # Check if drone starts in forbidden zone (from OptiTrack at very beginning)
    # We only care about the boolean result here, so we take the first element of the tuple.
    if point_in_polygon(x_initial_system_start, z_initial_system_start, forbidden_polygon)[0]:
        print(
            f"⚠️ Drone is inside forbidden region at script startup ({x_initial_system_start:.2f}, {z_initial_system_start:.2f}). Aborting mission and attempting to land.")
        send_command("land", timeout=10.0)
        time.sleep(5)
        script_exit_event.set()  # Signal immediate exit
    else:
        print(
            f"Drone starting position ({x_initial_system_start:.2f}, {z_initial_system_start:.2f}) is safe for script start. Proceeding.")
        if tello_startup():  # Only proceed if Tello initializes successfully and takes off
            # AFTER takeoff and stabilization, get current position for desired trajectory calculation
            with lock:
                x_after_takeoff = drone_position["x"]
                z_after_takeoff = drone_position["z"]
            calculate_desired_trajectory(x_after_takeoff, z_after_takeoff, initial_heading_degrees=0)

            # NOW start the monitor and patrol threads, after drone is airborne and position is stable
            monitor_thread.start()
            patrol_thread.start()
        else:
            print("Tello startup failed, not starting patrol.")
            script_exit_event.set()  # Signal exit if Tello fails

    try:
        # Main thread keeps running, waiting for the exit signal from other threads
        while not script_exit_event.is_set():
            time.sleep(1)
        print("Script exit event set. Initiating final cleanup.")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected (Ctrl+C). Signalling emergency land and cleanup.")
        emergency_land_event.set()  # Signal emergency on Ctrl+C
        send_command("land", timeout=10.0)  # Attempt to land immediately
        time.sleep(5)  # Give time for land command to execute
        script_exit_event.set()  # Signal all threads to exit

    finally:
        print("Attempting to join threads for graceful shutdown...")
        # Join non-daemon threads to allow them to complete their final tasks
        if monitor_thread.is_alive():
            print("Joining monitor_thread...")
            monitor_thread.join(timeout=10)
        if patrol_thread.is_alive():
            print("Joining patrol_thread...")
            patrol_thread.join(timeout=10)

        script_exit_event.set()  # Ensure plot thread receives exit signal
        if plot_thread.is_alive():
            print("Joining plot_thread...")
            plot_thread.join(timeout=10)

        print("All active threads should be terminated or joined.")

        if not sock._closed:  # Check if socket is still open
            print("Closing Tello UDP socket.")
            sock.close()
        else:
            print("Tello UDP socket was already closed.")

        # --- Generate final plot here ---
        # Pass the collected data to the final plotting function
        final_plot_trajectory(trajectory, emergency_escape_path, desired_trajectory_points, forbidden_polygon)

    print("Script execution fully complete. Goodbye!")
    sys.exit(0)  # Explicitly exit the script
