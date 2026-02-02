
import csv
import math
import sys

def calculate_oscillation_metrics(filepath):
    heading_errors = []
    cmd_omegas = []
    timestamps = []
    
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                # Filter for when MPC is active
                if row['active_ctrl'] == 'MPC':
                    heading_errors.append(float(row['heading_error_deg']))
                    cmd_omegas.append(float(row['cmd_omega']))
                    timestamps.append(float(row['timestamp']))
            except (ValueError, KeyError):
                continue
                
    if not heading_errors:
        print("No MPC data found.")
        return

    # 1. Sign Changes (Zero-crossings)
    def count_sign_changes(data):
        count = 0
        for i in range(1, len(data)):
            if data[i] * data[i-1] < 0:
                count += 1
        return count

    heading_sign_changes = count_sign_changes(heading_errors)
    omega_sign_changes = count_sign_changes(cmd_omegas)
    
    duration = timestamps[-1] - timestamps[0] if timestamps else 0
    freq_heading = heading_sign_changes / duration if duration > 0 else 0
    freq_omega = omega_sign_changes / duration if duration > 0 else 0

    # 2. Standard Deviation
    def calculate_std_dev(data):
        mean = sum(data) / len(data)
        variance = sum([((x - mean) ** 2) for x in data]) / len(data)
        return math.sqrt(variance)

    std_heading = calculate_std_dev(heading_errors)
    std_omega = calculate_std_dev(cmd_omegas)

    # 3. Mean Absolute Error
    mae_heading = sum([abs(x) for x in heading_errors]) / len(heading_errors)

    print(f"File: {filepath}")
    print(f"Duration with MPC: {duration:.2f}s")
    print(f"Heading Error Sign Changes: {heading_sign_changes} ({freq_heading:.2f} Hz)")
    print(f"Command Omega Sign Changes: {omega_sign_changes} ({freq_omega:.2f} Hz)")
    print(f"Heading Error Std Dev: {std_heading:.2f} deg")
    print(f"Command Omega Std Dev: {std_omega:.2f} rad/s")
    print(f"Mean Absolute Heading Error: {mae_heading:.2f} deg")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_oscillation.py <csv_file>")
    else:
        calculate_oscillation_metrics(sys.argv[1])
