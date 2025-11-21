import re
import struct
import pandas as pd

logfile = "sd_debug/out.txt"
out_csv = "out.csv"

acc_x, acc_y, acc_z = [], [], []
gyro_x, gyro_y, gyro_z = [], [], []
block_indices = []

array_re = re.compile(r"\[([0-9,\s]+)\]")

with open(logfile, "r", encoding="utf-8", errors="ignore") as f:
    for lineno, line in enumerate(f):
        m = array_re.search(line)
        if not m:
            continue
        num_str = m.group(1)
        try:
            arr = [int(x.strip()) for x in num_str.split(",")]
        except Exception as e:
            print(f"[line {lineno}] failed to parse ints: {e}")
            continue

        if len(arr) < 512:
            # skip incomplete blocks
            continue

        block = arr[:512]
        block_index = int.from_bytes(bytes(block[0:4]), "little")
        data_bytes = block[4:508]

        for i in range(0, len(data_bytes), 24):
            chunk = data_bytes[i : i + 24]
            if len(chunk) != 24:
                continue
            ax, ay, az, gx, gy, gz = struct.unpack("<ffffff", bytes(chunk))
            acc_x.append(ax)
            acc_y.append(ay)
            acc_z.append(az)
            gyro_x.append(gx)
            gyro_y.append(gy)
            gyro_z.append(gz)
            block_indices.append(block_index)

df = pd.DataFrame(
    {
        "BlockIndex": block_indices,
        "AccX": acc_x,
        "AccY": acc_y,
        "AccZ": acc_z,
        "GyroX": gyro_x,
        "GyroY": gyro_y,
        "GyroZ": gyro_z,
    }
)

df.to_csv(out_csv, index=False)
print(f"Saved {len(df)} samples to {out_csv}")
