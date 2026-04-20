import serial
import csv
from datetime import datetime

# ========= CONFIG =========
PORT = 'COM3'
BAUD = 115200
NUM_MOTORS = 2
OUTPUT_FILE = f"motor_log_{datetime.now().strftime('%H%M%S')}.csv"

# ========= LEADER-FOLLOWER GROUPING =========
leader_map = {
    0: None,
    1: 0,
}

def label_motor(i):
    if leader_map.get(i) is None:
        return f"L{i+1}"
    else:
        return f"F{i+1}"

# ========= SIGN CONVENTION FIX =========
sign_map = {
    0: 1,
    1: -1,
}

# ========= FIND FIRST LEADER/FOLLOWER =========
leader_idx = None
follower_idx = None

for i in range(NUM_MOTORS):
    if leader_map.get(i) is None and leader_idx is None:
        leader_idx = i
    elif leader_map.get(i) is not None and follower_idx is None:
        follower_idx = i

# ========= SERIAL =========
ser = serial.Serial(PORT, BAUD, timeout=1)

# ========= CSV SETUP =========
headers = ["time_ms"] + [label_motor(i) for i in range(NUM_MOTORS)] + ["error"]

# ========= NORMALIZATION STORAGE =========
initial_values = None

with open(OUTPUT_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(headers)

    print(f"Logging to {OUTPUT_FILE}...\n")

    while True:
        try:
            line = ser.readline().decode(errors='ignore').strip()

            if not line:
                continue

            parts = line.split(",")

            if len(parts) != NUM_MOTORS + 1:
                continue

            values = [int(x) for x in parts]

            # Capture reference
            if initial_values is None:
                initial_values = values.copy()
                print("Reference captured (normalized to zero)")
                continue

            # Normalize + fix direction
            normalized = [values[0] - initial_values[0]]

            corrected_vals = []

            for i in range(1, len(values)):
                motor_idx = i - 1
                corrected = (values[i] - initial_values[i]) * sign_map.get(motor_idx, 1)
                corrected_vals.append(corrected)
                normalized.append(corrected)

            # Tracking error (leader - follower)
            error = 0
            if leader_idx is not None and follower_idx is not None:
                error = corrected_vals[leader_idx] - corrected_vals[follower_idx]

            normalized.append(error)

            writer.writerow(normalized)
            print(normalized)

        except KeyboardInterrupt:
            print("\nStopped logging.")
            break