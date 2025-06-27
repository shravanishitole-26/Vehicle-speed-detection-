import serial
import tkinter as tk
import threading
import time
import winsound  # Windows-only module for sound
from collections import deque
import statistics

# Replace with the correct COM port
SERIAL_PORT = "COM5"  # Update to your actual port
BAUD_RATE = 115200

# Connect to TFmini-S
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)  # Ultra-low timeout
    print(f"Connected to {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"Error: {e}")
    exit()

# GUI Setup
root = tk.Tk()
root.title("Velocity Display with Alarm")
root.geometry("600x350")
root.configure(bg="black")

distance_label = tk.Label(root, text="Distance: -- cm", font=("Arial", 40, "bold"), fg="white", bg="black")
distance_label.pack()

velocity_label = tk.Label(root, text="-- km/h", font=("Arial", 300, "bold"), fg="white", bg="black") 
velocity_label.pack()

# Variables for velocity calculation
previous_distance = None
previous_time = None
velocity_buffer = deque(maxlen=20)  # Larger moving average window (20 readings)
smoothed_velocity = 0  # Exponential moving average
alpha = 0.1  # Lower alpha for better stability
deadband = 0.2  # Ignore small fluctuations (< 0.2 km/h)
alarm_active = False  # To prevent multiple alarm triggers

def play_alarm():
    """Plays an alarm sound for 5 seconds."""
    global alarm_active
    if not alarm_active:
        alarm_active = True
        winsound.Beep(1000, 5000)  # Beep at 1000 Hz for 5 seconds
        alarm_active = False

def update_display(distance, velocity):
    """Updates the GUI with distance and velocity values and triggers alarm if needed."""
    distance_label.config(text=f"Distance: {distance} cm")
    
    # Determine background color based on velocity
    bg_color = "red" if velocity > 5 else "green"
    root.configure(bg=bg_color)
    distance_label.configure(bg=bg_color)
    velocity_label.configure(bg=bg_color)
    
    # Update velocity display
    velocity_label.config(text=f"{int(velocity)} km/h", fg="white")

    # Trigger alarm if velocity crosses 5 km/h
    if velocity > 5 and not alarm_active:
        threading.Thread(target=play_alarm, daemon=True).start()

def read_lidar():
    """Continuously reads distance from TFmini-S and updates the display."""
    global previous_distance, previous_time, smoothed_velocity

    while True:
        ser.reset_input_buffer()  # Flush old data

        if ser.in_waiting >= 9:
            data = ser.read(9)

            if data[0] == 0x59 and data[1] == 0x59:  # Check frame header
                current_distance = data[2] + (data[3] << 8)  # Convert bytes to distance in cm
                current_time = time.time()  # Get current time in seconds

                # Calculate velocity if previous values exist
                if previous_distance is not None and previous_time is not None:
                    delta_d = abs(current_distance - previous_distance)  # Distance change
                    delta_t = current_time - previous_time  # Time change

                    if delta_t > 0:  # Prevent division by zero
                        velocity_cm_s = delta_d / delta_t  # Velocity in cm/s
                        velocity_km_h = velocity_cm_s * 0.036  # Convert to km/h
                    else:
                        velocity_km_h = 0
                else:
                    velocity_km_h = 0  # No movement initially

                # Store velocity in buffer
                velocity_buffer.append(velocity_km_h)

                # Apply Median Filter (removes sudden spikes)
                filtered_velocity = statistics.median(velocity_buffer)

                # Compute Weighted Exponential Moving Average (WEMA)
                smoothed_velocity = alpha * filtered_velocity + (1 - alpha) * smoothed_velocity

                # Ignore very small fluctuations
                if abs(smoothed_velocity - velocity_km_h) < deadband:
                    smoothed_velocity = velocity_km_h

                # Update previous values
                previous_distance = current_distance
                previous_time = current_time

                root.after(0, update_display, current_distance, smoothed_velocity)  # Safe UI update

# Start LiDAR reading in a background thread
threading.Thread(target=read_lidar, daemon=True).start()

root.mainloop()