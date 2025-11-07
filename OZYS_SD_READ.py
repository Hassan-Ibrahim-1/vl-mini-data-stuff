import ast
import struct
import pandas as pd
from datetime import datetime

filename = input("Input filename: ")
file = open(filename, "r", encoding="utf-16")
lines = file.readlines()
adc1 = []
adc2 = []
adc3 = []
adc4 = []
timestamps = []
j = 0
adc_gain = 330 / 6.8
gauge_factor = 2
sg_voltage = 3.3
for line in lines:
    line_stripped = line.strip()
    if line_stripped.startswith("["):
        print(f"Line {j} ✅ starts with '['")
        arr = ast.literal_eval(line_stripped)
        timestamp_bytes = bytes(
            [arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7]]
        )
        ts = int.from_bytes(timestamp_bytes, "little", signed=True)
        dt = datetime.fromtimestamp(ts / 1_000_000)
        # print(ts)
        # print(dt)
        for i in range(8, 496, 16):
            sample1 = bytes([arr[i], arr[i + 1], arr[i + 2], arr[i + 3]])
            sample2 = bytes([arr[i + 4], arr[i + 5], arr[i + 6], arr[i + 7]])
            sample3 = bytes([arr[i + 8], arr[i + 9], arr[i + 10], arr[i + 11]])
            sample4 = bytes([arr[i + 12], arr[i + 13], arr[i + 14], arr[i + 15]])
            adc1.append(
                (struct.unpack("<f", sample1)[0])
                / (adc_gain * sg_voltage * gauge_factor)
            )
            adc2.append(
                (struct.unpack("<f", sample2)[0])
                / (adc_gain * sg_voltage * gauge_factor)
            )
            adc3.append(
                (struct.unpack("<f", sample3)[0])
                / (adc_gain * sg_voltage * gauge_factor)
            )
            adc4.append(
                (struct.unpack("<f", sample4)[0])
                / (adc_gain * sg_voltage * gauge_factor)
            )
            timestamps.append(dt)
    else:
        print(f"Line {j} ❌ does NOT start with '[' -> {line_stripped[:10]}")
    j += 1

df = pd.DataFrame(
    {
        "Recent Timestamp": timestamps,
        "ADC1 Strain": adc1,
        "ADC2 Strain": adc2,
        "ADC3 Strain": adc3,
        "ADC4 Strain": adc4,
    }
)

print(df)
df.to_csv("output.csv", index=False)
print(
    "✅ CSV file 'adc_log.csv' created successfully! From ",
    df.shape[0] / 31.0,
    "Valid Rows",
)  # 31 samples per arr/block

# 6 readings, 3 gyro 3 acceleration
# need a vl mini probe

