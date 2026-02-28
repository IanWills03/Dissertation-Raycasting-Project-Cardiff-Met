import threading
import time
import socket
import sys
from collections import deque
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from MotionCapture_Files.NatNetClient import NatNetClient

drone_position = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
trajectory = []
lock = threading.Lock()
emergency_land_event = threading.Event()
script_exit_event = threading.Event()
retreated_to_safe_distance = threading.Event()

fig = None
ax = None
position_text = None
emergency_status_text = None

emergency_entry_point = None
emergency_escape_path = []
desired_trajectory_points = []

ray_start_plot = None
ray_end_plot = None
intersection_points_plot = []

forbidden_polygon = [
    (0.8, 0.3), (1.3, 0.55), (1.3, 1.05),
    (0.8, 1.3),  (0.3, 1.05),(0.3, 0.55)
]

WARNING_MARGIN = 0.5

TELLO_IP = '192.168.10.1'
TELLO_PORT = 8889
BUFFER_SIZE = 1024
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
tello_address = (TELLO_IP, TELLO_PORT)


def point_in_polygon(x, y, polygon):
    num = len(polygon)
    j = num - 1
    inside = False
    intersections = []

    x_far = max(p[0] for p in polygon) + 5.0
    ray_start = (x, y)
    ray_end = (x_far, y)

    for i in range(num):
        p1 = polygon[i]
        p2 = polygon[j]

        if ((p1[1] > y) != (p2[1] > y)):
            x_intersect = (p2[0] - p1[0]) * (y - p1[1]) / ((p2[1] - p1[1]) + 1e-10) + p1[0]

            if x_intersect >= x and (min(p1[0], p2[0]) <= x_intersect <= max(p1[0], p2[0])):
                inside = not inside
                intersections.append((x_intersect, y))
        j = i
    return inside, ray_start, ray_end, intersections


def distance_point_to_segment(px, py, x1, y1, x2, y2):
    line_mag_sq = (x2 - x1) ** 2 + (y2 - y1) ** 2
    if line_mag_sq == 0:
        return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)

    t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / line_mag_sq

    if t < 0:
        closest_x, closest_y = x1, y1
    elif t > 1:
        closest_x, closest_y = x2, y2
    else:
        closest_x = x1 + t * (x2 - x1)
        closest_y = y1 + t * (y2 - y1)

    return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)


def is_within_warning_zone(x, z, polygon, margin):
    min_distance = float('inf')
    num_vertices = len(polygon)

    for i in range(num_vertices):
        p1_x, p1_z = polygon[i]
        p2_x, p2_z = polygon[(i + 1) % num_vertices]

        dist = distance_point_to_segment(x, z, p1_x, p1_z, p2_x, p2_z)
        min_distance = min(min_distance, dist)

    return min_distance <= margin


def send_command(command, timeout=2.0):
    if emergency_land_event.is_set() and not (command.startswith("back") or command in ["land", "stop", "reboot"]):
        print(f"⚠️ Emergency in progress. Skipping non-emergency command: '{command}'")
        return None

    if sock._closed:
        print(f"⚠️ Socket is closed. Cannot send command: '{command}'")
        return None

    try:
        print(f"Sending command: '{command}'")
        sock.sendto(command.encode('utf-8'), tello_address)
        sock.settimeout(timeout)
        msg, _ = sock.recvfrom(BUFFER_SIZE)
        response = msg.decode('utf-8').strip()
        print(f"Response: {response}")

        if "error" in response.lower() or "motor stop" in response.lower() or "no valid imu" in response.lower():
            print(f"🚨 CRITICAL TELLO ERROR: '{response}'. Initiating emergency land and script shutdown.")
            emergency_land_event.set()
            script_exit_event.set()
            try:
                sock.sendto("land".encode('utf-8'), tello_address)
            except Exception as e:
                print(f"Error sending final land command: {e}")
            return False  # Indicate a critical error occurred

        return True  # Indicate success
    except socket.timeout:
        print(f"No response from Tello for command: '{command}' (timeout: {timeout}s)")
        print(f"🚨 TELLO TIMEOUT: No response for '{command}'. Initiating emergency land and script shutdown.")
        emergency_land_event.set()
        script_exit_event.set()
        return False
    except Exception as e:
        print(f"⚠️ Command failed for '{command}' | Error: {e}")
        print(f"🚨 TELLO COMMUNICATION ERROR: '{e}'. Initiating emergency land and script shutdown.")
        emergency_land_event.set()
        script_exit_event.set()
        return False


def receive_rigid_body_position(rigid_body_id, position, rotation):
    global drone_position
    if rigid_body_id == 1:
        with lock:
            drone_position["x"] = position[0]
            drone_position["y"] = position[1]
            drone_position["z"] = position[2]

            w = rotation[0]
            x = rotation[1]
            y = rotation[2]
            z = rotation[3]

            siny_cosp = +2.0 * (w * y + z * x)
            cosy_cosp = +1.0 - 2.0 * (y * y + x * x)
            yaw_rad = math.atan2(siny_cosp, cosy_cosp)

            drone_position["yaw"] = math.degrees(yaw_rad)


def receive_position():
    print("Starting NatNet client to receive position data...")
    client = NatNetClient()
    client.rigid_body_listener = receive_rigid_body_position
    try:
        client.run()
    except Exception as e:
        print(f"Error in NatNet client thread: {e}")
        script_exit_event.set()


def set_emergency_status_text(status_text):
    global emergency_status_text
    if emergency_status_text:
        emergency_status_text.set_text(status_text)
        if "EMERGENCY" in status_text:
            emergency_status_text.set_color('red')
        elif "WARNING" in status_text:
            emergency_status_text.set_color('orange')
        else:
            emergency_status_text.set_color('blue')


def plot_trajectory():
    global trajectory, fig, ax, emergency_entry_point, emergency_escape_path, desired_trajectory_points, position_text, emergency_status_text, ray_start_plot, ray_end_plot, intersection_points_plot

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))

    forbidden_patch = patches.Polygon(forbidden_polygon, closed=True, edgecolor='red', facecolor='red', alpha=0.2,
                                      linewidth=2,
                                      label="Forbidden Zone")
    ax.add_patch(forbidden_patch)

    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Z (meters)")
    ax.set_title("Live Drone Trajectory (Top-Down View)")

    desired_path_line, = ax.plot([], [], 'k--', label="Desired Path", alpha=0.7, linewidth=2)

    drone_path_line, = ax.plot([], [], 'b-', label="Actual Path (Normal)", alpha=0.8, markersize=5, linewidth=3)

    escape_path_line, = ax.plot([], [], 'g-', label="Emergency Escape Path", alpha=0.9, markersize=7, linewidth=4)
    entry_point_marker, = ax.plot([], [], 'rX', markersize=20, label="Entry Point to Forbidden Zone")

    ray_line, = ax.plot([], [], 'c:', label="Ray Cast", alpha=0.5, linewidth=1)
    intersection_markers, = ax.plot([], [], 'ro', markersize=8, label="Ray Intersections")

    ax.legend()
    ax.set_aspect('equal', adjustable='box')

    min_x_poly = min(p[0] for p in forbidden_polygon)
    max_x_poly = max(p[0] for p in forbidden_polygon)
    min_z_poly = min(p[1] for p in forbidden_polygon)
    max_z_poly = max(p[1] for p in forbidden_polygon)

    padding_initial = 0.5
    ax.set_xlim(min_x_poly - padding_initial, max_x_poly + padding_initial)
    ax.set_ylim(max_z_poly + padding_initial, min_z_poly - padding_initial)

    position_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                            fontsize=10, verticalalignment='top',
                            bbox=dict(boxstyle="round,pad=0.3", fc="cyan", ec="b", lw=1, alpha=0.6))

    emergency_status_text = ax.text(0.02, 0.93, 'Status: Normal', transform=ax.transAxes,
                                    fontsize=10, verticalalignment='top', color='blue',
                                    bbox=dict(boxstyle="round,pad=0.3", fc="yellow", ec="darkblue", lw=1, alpha=0.6))

    window_size = 20
    smoothed_x_queue = deque(maxlen=window_size)
    smoothed_z_queue = deque(maxlen=window_size)

    while not script_exit_event.is_set():
        with lock:
            x = drone_position["x"]
            z = drone_position["z"]
            altitude = drone_position["y"]
            is_emergency = emergency_land_event.is_set()
            has_retreated_safely = retreated_to_safe_distance.is_set()

            current_ray_start = ray_start_plot
            current_ray_end = ray_end_plot
            current_intersections = intersection_points_plot

        current_pos = (x, z)

        if position_text:
            position_text.set_text(f'Pos: X={x:.2f}m, Z={z:.2f}m, Y(Alt)={altitude:.2f}m')

        is_in_forbidden_zone_current, _, _, _ = point_in_polygon(x, z, forbidden_polygon)
        is_in_warning_zone_current = is_within_warning_zone(x, z, forbidden_polygon, WARNING_MARGIN)

        status_msg = "Status: Normal"
        current_path_color = 'b'

        if is_in_forbidden_zone_current:
            status_msg = "EMERGENCY: IN FORBIDDEN ZONE!"
            current_path_color = 'r'
        elif is_in_warning_zone_current and not is_in_forbidden_zone_current:
            status_msg = "WARNING: APPROACHING FORBIDDEN ZONE!"
            current_path_color = 'orange'
        elif is_emergency and not has_retreated_safely:
            status_msg = "EMERGENCY: EVADING!"
            current_path_color = 'g'
        elif has_retreated_safely:
            status_msg = "EMERGENCY: RETREATED SAFELY!"
            current_path_color = 'purple'

        if current_ray_start is not None:
            num_intersections = len(current_intersections)
            if num_intersections % 2 == 1:
                status_msg += f" (Inside Polygon: {num_intersections} intersections)"
            else:
                status_msg += f" (Outside Polygon: {num_intersections} intersections)"

        set_emergency_status_text(status_msg)

        smoothed_x_queue.append(x)
        smoothed_z_queue.append(z)

        if not is_emergency and not has_retreated_safely:
            trajectory.append(current_pos)
            if emergency_escape_path or emergency_entry_point:
                emergency_escape_path.clear()
                emergency_entry_point = None
        elif is_emergency:
            emergency_escape_path.append(current_pos)

        if len(smoothed_x_queue) == window_size:
            avg_x = np.mean(list(smoothed_x_queue))
            avg_z = np.mean(list(smoothed_z_queue))

            if not is_emergency and not has_retreated_safely and trajectory:
                trajectory[-1] = (avg_x, avg_z)
            elif is_emergency and emergency_escape_path:
                emergency_escape_path[-1] = (avg_x, avg_z)

        x_vals_normal, z_vals_normal = zip(*trajectory) if trajectory else ([], [])
        drone_path_line.set_data(list(x_vals_normal), list(z_vals_normal))
        drone_path_line.set_color(current_path_color)

        if emergency_escape_path:
            x_vals_escape, z_vals_escape = zip(*emergency_escape_path)
            escape_path_line.set_data(list(x_vals_escape), list(z_vals_escape))
        else:
            escape_path_line.set_data([], [])

        if desired_trajectory_points:
            desired_x_vals, desired_z_vals = zip(*desired_trajectory_points)
            desired_path_line.set_data(list(desired_x_vals), list(desired_z_vals))

        if emergency_entry_point:
            entry_point_marker.set_data([emergency_entry_point[0]], [emergency_entry_point[1]])
        else:
            entry_point_marker.set_data([], [])

        if current_ray_start and current_ray_end:
            ray_line.set_data([current_ray_start[0], current_ray_end[0]], [current_ray_start[1], current_ray_end[1]])
        else:
            ray_line.set_data([], [])

        if current_intersections:
            intersect_x = [p[0] for p in current_intersections]
            intersect_z = [p[1] for p in current_intersections]
            intersection_markers.set_data(intersect_x, intersect_z)
        else:
            intersection_markers.set_data([], [])

        all_x_coords = [p[0] for p in trajectory + emergency_escape_path + desired_trajectory_points if p is not None]
        all_z_coords = [p[1] for p in trajectory + emergency_escape_path + desired_trajectory_points if p is not None]

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

            all_x_coords.extend([p[0] for p in forbidden_polygon])
            all_z_coords.extend([p[1] for p in forbidden_polygon])

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

            ax.set_xlim(center_x - max_range / 2, center_x + max_range / 2)
            ax.set_ylim(center_z + max_range / 2, center_z - max_range / 2)

        try:
            plt.pause(0.001)
        except Exception as e:
            print(f"Plotting update error (likely GUI closed): {e}")
            break

    if fig is not None and plt.fignum_exists(fig.number):
        plt.close(fig)
        print("Plot window closed by script.")


def monitor_and_react():
    global emergency_entry_point, emergency_escape_path, ray_start_plot, ray_end_plot, intersection_points_plot, retreated_to_safe_distance
    has_landed = False

    while not has_landed and not script_exit_event.is_set():
        with lock:
            x = drone_position["x"]
            z = drone_position["z"]

        is_in_forbidden_zone, current_ray_start, current_ray_end, current_intersections = point_in_polygon(x, z,
                                                                                                           forbidden_polygon)
        is_in_warning_zone = is_within_warning_zone(x, z, forbidden_polygon, WARNING_MARGIN)

        with lock:
            ray_start_plot = current_ray_start
            ray_end_plot = current_ray_end
            intersection_points_plot = current_intersections

            if is_in_forbidden_zone and not emergency_land_event.is_set():
                emergency_land_event.set()
                print(f"🚨 EMERGENCY TRIGGERED at ({x:.2f}, {z:.2f})! Reason: IN FORBIDDEN ZONE.")
            elif is_in_warning_zone and not is_in_forbidden_zone and not emergency_land_event.is_set():
                print(f"⚠️ WARNING: Drone at ({x:.2f}, {z:.2f}) is in WARNING ZONE (not forbidden).")

        if emergency_land_event.is_set() and not has_landed:
            print("🚨 Drone in forbidden region! Initiating emergency actions.")

            if emergency_entry_point is None:
                emergency_entry_point = (x, z)
                emergency_escape_path.clear()

            print("Sending 'stop' command to halt current movement.")
            if not send_command("stop", timeout=2.0): return  # Exit if stop fails
            time.sleep(1)

            retreat_distance_cm = 300
            segment_size_cm = 50
            current_retreated_cm = 0

            while (point_in_polygon(drone_position["x"], drone_position["z"], forbidden_polygon)[0] or \
                   is_within_warning_zone(drone_position["x"], drone_position["z"], forbidden_polygon,
                                          WARNING_MARGIN)) and \
                    current_retreated_cm < retreat_distance_cm and not script_exit_event.is_set():

                dist_to_move = min(segment_size_cm, retreat_distance_cm - current_retreated_cm)
                print(f"Executing retreat segment: back {dist_to_move}cm.")
                if not send_command(f"back {dist_to_move}", timeout=5.0): return  # Exit if retreat segment fails
                time.sleep(2)

                current_retreated_cm += dist_to_move

            with lock:
                current_x_after_retreat = drone_position["x"]
                current_z_after_retreat = drone_position["z"]

            is_out_of_warning_zone_after_retreat = not is_within_warning_zone(current_x_after_retreat,
                                                                              current_z_after_retreat,
                                                                              forbidden_polygon, WARNING_MARGIN)
            is_out_of_forbidden_zone_after_retreat = not \
            point_in_polygon(current_x_after_retreat, current_z_after_retreat, forbidden_polygon)[0]

            if is_out_of_warning_zone_after_retreat and is_out_of_forbidden_zone_after_retreat:
                print(
                    f"✅ Drone successfully moved out of warning zone AND forbidden zone after decisive retreat ({current_x_after_retreat:.2f}, {current_z_after_retreat:.2f}).")
                retreated_to_safe_distance.set()
            else:
                current_status_after_retreat = []
                if not is_out_of_forbidden_zone_after_retreat:
                    current_status_after_retreat.append("FORBIDDEN ZONE")
                if not is_out_of_warning_zone_after_retreat:
                    current_status_after_retreat.append("WARNING ZONE")

                print(
                    f"⚠️ Despite decisive retreat, still in {' and '.join(current_status_after_retreat)} ({current_x_after_retreat:.2f}, {current_z_after_retreat:.2f}). This indicates a significant issue or very rapid drone movement.")

            print("Proceeding to land.")
            if not send_command("land", timeout=10.0): return  # Exit if land fails
            time.sleep(5)
            has_landed = True
            script_exit_event.set()
            break

        time.sleep(0.1)


def calculate_desired_trajectory(start_x, start_z, initial_heading_degrees=0):
    global desired_trajectory_points
    desired_trajectory_points.clear()

    current_x = start_x
    current_z = start_z
    current_heading_rad = math.radians(initial_heading_degrees)

    desired_trajectory_points.append((current_x, current_z))

    segment_size_cm = 50

    for i in range(5):
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

        current_heading_rad += math.radians(90)
        desired_trajectory_points.append((current_x, current_z))

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

        current_heading_rad += math.radians(90)
        desired_trajectory_points.append((current_x, current_z))

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

        current_heading_rad -= math.radians(90)
        desired_trajectory_points.append((current_x, current_z))

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

        current_heading_rad -= math.radians(90)
        desired_trajectory_points.append((current_x, current_z))

    print("Desired trajectory calculated.")


def patrolling():
    print("Patrolling the room...")
    command_execution_delay = 1.0
    command_timeout = 5.0
    segment_size_cm = 50

    for i in range(5):
        if emergency_land_event.is_set() or script_exit_event.is_set():
            print("Patrol aborted due to emergency or script exit.")
            return  # Exit function immediately

        print(f"🌀 Patrol iteration {i + 1}")

        total_forward_dist = 280
        current_moved = 0
        while current_moved < total_forward_dist and not emergency_land_event.is_set() and not script_exit_event.is_set():
            dist_to_move = min(segment_size_cm, total_forward_dist - current_moved)
            if not send_command(f"forward {dist_to_move}", timeout=command_timeout):
                return  # Exit function immediately on command failure
            if emergency_land_event.wait(command_execution_delay) or script_exit_event.is_set():
                print(f"Patrol interrupted during forward {dist_to_move} execution.")
                return  # Exit function immediately
            current_moved += dist_to_move
        if emergency_land_event.is_set() or script_exit_event.is_set(): return

        if not send_command("ccw 90", timeout=command_timeout): return
        if emergency_land_event.wait(command_execution_delay) or script_exit_event.is_set():
            print("Patrol interrupted during ccw 90 execution.")
            return

        total_forward_dist = 50
        current_moved = 0
        while current_moved < total_forward_dist and not emergency_land_event.is_set() and not script_exit_event.is_set():
            dist_to_move = min(segment_size_cm, total_forward_dist - current_moved)
            if not send_command(f"forward {dist_to_move}", timeout=command_timeout):
                return
            if emergency_land_event.wait(command_execution_delay) or script_exit_event.is_set():
                print(f"Patrol interrupted during forward {dist_to_move} execution.")
                return
            current_moved += dist_to_move
        if emergency_land_event.is_set() or script_exit_event.is_set(): return

        if not send_command("ccw 90", timeout=command_timeout): return
        if emergency_land_event.wait(command_execution_delay) or script_exit_event.is_set():
            print("Patrol interrupted during ccw 90 execution.")
            return

        total_forward_dist = 280
        current_moved = 0
        while current_moved < total_forward_dist and not emergency_land_event.is_set() and not script_exit_event.is_set():
            dist_to_move = min(segment_size_cm, total_forward_dist - current_moved)
            if not send_command(f"forward {dist_to_move}", timeout=command_timeout):
                return
            if emergency_land_event.wait(command_execution_delay) or script_exit_event.is_set():
                print(f"Patrol interrupted during forward {dist_to_move} execution.")
                return
            current_moved += dist_to_move
        if emergency_land_event.is_set() or script_exit_event.is_set(): return

        if not send_command("cw 90", timeout=command_timeout): return
        if emergency_land_event.wait(command_execution_delay) or script_exit_event.is_set():
            print("Patrol interrupted during cw 90 execution.")
            return

        total_forward_dist = 50
        current_moved = 0
        while current_moved < total_forward_dist and not emergency_land_event.is_set() and not script_exit_event.is_set():
            dist_to_move = min(segment_size_cm, total_forward_dist - current_moved)
            if not send_command(f"forward {dist_to_move}", timeout=command_timeout):
                return
            if emergency_land_event.wait(command_execution_delay) or script_exit_event.is_set():
                print(f"Patrol interrupted during forward {dist_to_move} execution.")
                return
            current_moved += dist_to_move
        if emergency_land_event.is_set() or script_exit_event.is_set(): return

        if not send_command("cw 90", timeout=command_timeout): return
        if emergency_land_event.wait(command_execution_delay) or script_exit_event.is_set():
            print("Patrol interrupted during cw 90 execution.")
            return

    if not emergency_land_event.is_set() and not script_exit_event.is_set():
        print("Patrolling sequence completed.")


def patrol_thread_func():
    print("Patrol thread starting...")
    patrolling()
    print("Patrol thread finished.")


def tello_startup():
    print("Initializing Tello...")
    if not send_command("command", timeout=5.0): return False

    if not send_command("battery?", timeout=2.0): return False

    takeoff_response_ok = send_command("takeoff", timeout=10.0)
    if not takeoff_response_ok: return False

    if takeoff_response_ok:
        if not send_command("up 30", timeout=5.0): return False
        time.sleep(3)
        print("Tello is now flying and stabilized.")
        return True
    else:
        print("Tello takeoff failed. Response:", takeoff_response_ok)
        script_exit_event.set()
        return False


def final_plot_trajectory(final_trajectory, final_emergency_escape_path, final_desired_trajectory_points,
                          final_forbidden_polygon):
    print("Generating final trajectory plot...")
    fig_final, ax_final = plt.subplots(figsize=(10, 10))

    forbidden_patch_final = patches.Polygon(final_forbidden_polygon, closed=True, edgecolor='red', facecolor='red',
                                            alpha=0.2, linewidth=2,
                                            label="Forbidden Zone")
    ax_final.add_patch(forbidden_patch_final)

    ax_final.set_xlabel("X (meters)")
    ax_final.set_ylabel("Z (meters)")
    ax_final.set_title("Final Drone Trajectory (Top-Down View)")

    if final_desired_trajectory_points:
        desired_x_vals, desired_z_vals = zip(*final_desired_trajectory_points)
        ax_final.plot(list(desired_x_vals), list(desired_z_vals), 'k--', label="Desired Path", alpha=0.7, linewidth=2)

    if final_trajectory:
        x_vals_normal, z_vals_normal = zip(*final_trajectory)
        ax_final.plot(list(x_vals_normal), list(z_vals_normal), 'b-', label="Actual Path (Normal)", alpha=0.8,
                      markersize=5, linewidth=3)

    if final_emergency_escape_path:
        x_vals_escape, z_vals_escape = zip(*final_emergency_escape_path)
        ax_final.plot(list(x_vals_escape), list(z_vals_escape), 'g-', label="Emergency Escape Path", alpha=0.9,
                      markersize=7, linewidth=4)

        if emergency_entry_point:
            ax_final.plot(emergency_entry_point[0], emergency_entry_point[1], 'rX', markersize=20,
                          label="Entry Point to Forbidden Zone")

    ax_final.legend()
    ax_final.set_aspect('equal', adjustable='box')

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
        ax_final.set_ylim(center_z + max_range / 2, center_z - max_range / 2)

    plt.show(block=True)
    print("Final plot displayed.")


if __name__ == "__main__":
    print("Starting drone control script...")

    emergency_land_event.clear()
    script_exit_event.clear()
    retreated_to_safe_distance.clear()

    natnet_thread = threading.Thread(target=receive_position, daemon=True)
    plot_thread = threading.Thread(target=plot_trajectory)
    monitor_thread = threading.Thread(target=monitor_and_react)
    patrol_thread = threading.Thread(target=patrol_thread_func)

    natnet_thread.start()
    plot_thread.start()

    print("Waiting for motion capture system to establish initial position...")
    time.sleep(5)

    with lock:
        x_initial_system_start = drone_position["x"]
        z_initial_system_start = drone_position["z"]

    if point_in_polygon(x_initial_system_start, z_initial_system_start, forbidden_polygon)[0]:
        print(
            f"⚠️ Drone is inside forbidden region at script startup ({x_initial_system_start:.2f}, {z_initial_system_start:.2f}). Aborting mission and attempting to land.")
        send_command("land", timeout=10.0)
        time.sleep(5)
        script_exit_event.set()
    else:
        print(
            f"Drone starting position ({x_initial_system_start:.2f}, {z_initial_system_start:.2f}) is safe for script start. Proceeding.")
        if tello_startup():
            with lock:
                x_after_takeoff = drone_position["x"]
                z_after_takeoff = drone_position["z"]
            calculate_desired_trajectory(x_after_takeoff, z_after_takeoff, initial_heading_degrees=0)

            monitor_thread.start()
            patrol_thread.start()
        else:
            print("Tello startup failed, not starting patrol.")
            script_exit_event.set()

    try:
        while not script_exit_event.is_set():
            time.sleep(1)
        print("Script exit event set. Initiating final cleanup.")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected (Ctrl+C). Signalling emergency land and cleanup.")
        emergency_land_event.set()
        send_command("land", timeout=10.0)
        time.sleep(5)
        script_exit_event.set()

    finally:
        print("Attempting to join threads for graceful shutdown...")
        if monitor_thread.is_alive():
            print("Joining monitor_thread...")
            monitor_thread.join(timeout=10)
        if patrol_thread.is_alive():
            print("Joining patrol_thread...")
            patrol_thread.join(timeout=10)

        script_exit_event.set()
        if plot_thread.is_alive():
            print("Joining plot_thread...")
            plot_thread.join(timeout=10)

        print("All active threads should be terminated or joined.")

        if not sock._closed:
            print("Closing Tello UDP socket.")
            sock.close()
        else:
            print("Tello UDP socket was already closed.")

        final_plot_trajectory(trajectory, emergency_escape_path, desired_trajectory_points, forbidden_polygon)

    print("Script execution fully complete. Goodbye!")
    sys.exit(0)
