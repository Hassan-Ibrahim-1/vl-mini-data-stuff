import ast
import struct
import pandas as pd
import re

filename = input("Input filename: ")
file = open(filename, "r")
lines = file.readlines()

acc_x = []
acc_y = []
acc_z = []
gyro_x = []
gyro_y = []
gyro_z = []
log_timestamps = []

j = 0
samples_per_block = 20  # (504 - 8) // 24 = 20 samples per block

for line in lines:
    line_stripped = line.strip()

    # Find the array portion using regex - matches [0, 128, ...]
    match = re.search(r"\[[\d,\s]+\]", line_stripped)

    if match:
        print(f"Line {j} ✅ found data array")
        arr_str = match.group(0)
        arr = ast.literal_eval(arr_str)

        # Extract log timestamp from beginning of line (e.g., "0.092626")
        log_ts_match = re.match(r"^([\d.]+)", line_stripped)
        log_ts = float(log_ts_match.group(1)) if log_ts_match else 0.0

        # Each sample is 24 bytes: 3 floats for acc + 3 floats for gyro
        # Data starts at byte 0, CRC is at bytes 508-511
        for i in range(0, samples_per_block * 24, 24):
            if i + 24 > len(arr):
                break

            # Acceleration (m/s^2) - 3 floats
            ax = struct.unpack("<f", bytes(arr[i : i + 4]))[0]
            ay = struct.unpack("<f", bytes(arr[i + 4 : i + 8]))[0]
            az = struct.unpack("<f", bytes(arr[i + 8 : i + 12]))[0]

            # Gyroscope (deg/s) - 3 floats
            gx = struct.unpack("<f", bytes(arr[i + 12 : i + 16]))[0]
            gy = struct.unpack("<f", bytes(arr[i + 16 : i + 20]))[0]
            gz = struct.unpack("<f", bytes(arr[i + 20 : i + 24]))[0]

            acc_x.append(ax)
            acc_y.append(ay)
            acc_z.append(az)
            gyro_x.append(gx)
            gyro_y.append(gy)
            gyro_z.append(gz)
            log_timestamps.append(log_ts)
    else:
        print(f"Line {j} ❌ no data array found -> {line_stripped[:40]}...")
    j += 1

df = pd.DataFrame(
    {
        "Log_Time (s)": log_timestamps,
        "Acc_X (m/s²)": acc_x,
        "Acc_Y (m/s²)": acc_y,
        "Acc_Z (m/s²)": acc_z,
        "Gyro_X (deg/s)": gyro_x,
        "Gyro_Y (deg/s)": gyro_y,
        "Gyro_Z (deg/s)": gyro_z,
    }
)

print(df)
df.to_csv("imu_output.csv", index=False)
print(f"✅ CSV file 'imu_output.csv' created successfully!")
print(f"   Total samples: {df.shape[0]}")
print(f"   From {df.shape[0] / samples_per_block:.1f} valid blocks")
