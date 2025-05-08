#!/usr/bin/env python3

import csv
import numpy as np
import matplotlib.pyplot as plt
import os

def load_data(vehicle_file, control_file):
    # Load vehicle state data
    vehicle_times = []
    vehicle_x = []
    vehicle_y = []
    vehicle_yaw = []
    with open(vehicle_file, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # Skip header
        for row in reader:
            if len(row) >= 4:  # 
                vehicle_times.append(float(row[0]))
                # Strip brackets and convert to float for X, Y, Yaw
                vehicle_x.append(float(row[1].strip('[]')))
                vehicle_y.append(float(row[2].strip('[]')))
                vehicle_yaw.append(float(row[3].strip('[]')))
            else:
                print(f"Skipping invalid row in vehicle data: {row}")

    # Load control data
    control_times = []
    throttle = []
    steer = []
    brake = []
    with open(control_file, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # Skip header
        for row in reader:
            if len(row) >= 4:  
                control_times.append(float(row[0]))
                throttle.append(float(row[1]))
                steer.append(float(row[2]))
                brake.append(float(row[3]))
            else:
                print(f"Skipping invalid row in control data: {row}")

    return (vehicle_times, vehicle_x, vehicle_y, vehicle_yaw), (control_times, throttle, steer, brake)

def plot_trajectory(vehicle_data):
    times, x, y, yaw = vehicle_data
    plt.figure(figsize=(10, 8))
    plt.plot(x, y, 'b-', label='Vehicle Trajectory')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Vehicle Trajectory in CARLA')
    plt.grid(True)
    plt.legend()
    plt.axis('equal')  # Equal aspect ratio for better visualization
    plt.show()

def plot_controls(control_data):
    times, throttle, steer, brake = control_data
    plt.figure(figsize=(10, 6))
    
    plt.subplot(3, 1, 1)
    plt.plot(times, throttle, 'g-', label='Throttle')
    plt.xlabel('Time (s)')
    plt.ylabel('Throttle')
    plt.title('Control Inputs Over Time')
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(times, steer, 'r-', label='Steer')
    plt.xlabel('Time (s)')
    plt.ylabel('Steer')
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(times, brake, 'b-', label='Brake')
    plt.xlabel('Time (s)')
    plt.ylabel('Brake')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Replace with your actual log file paths
    vehicle_file = os.path.expanduser("~/mpc_logs/vehicle_data_20250507_183439.csv")  # Update with correct timestamp from home directory mpc_logs or thhe place where it is saved
    control_file = os.path.expanduser("~/mpc_logs/control_data_20250507_183439.csv")  # Update with correct timestamp from home directory mpc_logs or thhe place where it is saved
    
    if not os.path.exists(vehicle_file) or not os.path.exists(control_file):
        print("Log files not found. Please update the file paths.")
        # Optionally, list available files in the directory for debugging
        log_dir = os.path.expanduser("~/mpc_logs")
        if os.path.exists(log_dir):
            print(f"Available files in {log_dir}:")
            for f in os.listdir(log_dir):
                print(f" - {f}")
    else:
        try:
            vehicle_data, control_data = load_data(vehicle_file, control_file)
            plot_trajectory(vehicle_data)
            plot_controls(control_data)
        except Exception as e:
            print(f"Error processing data: {e}")
