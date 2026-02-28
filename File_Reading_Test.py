import socket
import time

# Configuration parameters
FILENAME = "C:\\Users\\ianwi\\OneDrive - Cardiff Metropolitan University\\Dissertation Raycasting Project\\Text_file\\Data_Readings.txt"
TARGET_IP = "127.0.0.1"  # Change to the destination IP address
TARGET_PORT = 1511        # Destination port number
DELAY_BETWEEN_LINES = 0.1  # Delay (in seconds) between sending each line

def send_filtered_lines(filename, target_ip, target_port, delay):
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        with open(filename, 'r') as file:
            for line in file:
                # Remove any trailing newline characters
                line = line.rstrip("\n")

                # Send every line to the UDP socket
                sock.sendto(line.encode(), (target_ip, target_port))

                # Only print updated position lines
                if "Rigid Body 1: Position" in line:
                    print(f"Sent (Position Only): {line}")

                time.sleep(delay)  # Optional: wait before sending the next line
    except FileNotFoundError:
        print(f"Error: The file '{filename}' was not found.")
    finally:
        sock.close()

if __name__ == "__main__":
    send_filtered_lines(FILENAME, TARGET_IP, TARGET_PORT, DELAY_BETWEEN_LINES)
