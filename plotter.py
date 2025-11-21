import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load new IMU CSV
data = pd.read_csv("imu_output.csv")

# Drop rows that contain any zeros (optional — depends on your data)
data = data[(data != 0).all(axis=1)]

# Use row index as x-axis
x = data.index.to_numpy()

# Extract IMU channels
ax = data["AccX (m/s^2)"].to_numpy()
ay = data["AccY (m/s^2)"].to_numpy()
az = data["AccZ (m/s^2)"].to_numpy()

gx = data["GyroX (deg/s)"].to_numpy()
gy = data["GyroY (deg/s)"].to_numpy()
gz = data["GyroZ (deg/s)"].to_numpy()

print("Number of samples:", len(x))

# Plot accel signals
plt.figure()
plt.plot(x, ax, label="AccX")
plt.plot(x, ay, label="AccY")
plt.plot(x, az, label="AccZ")
plt.xlabel("Sample Index")
plt.ylabel("Acceleration (m/s^2)")
plt.title("Accelerometer Data")
plt.legend()
plt.show()

# Plot gyro signals
plt.figure()
plt.plot(x, gx, label="GyroX")
plt.plot(x, gy, label="GyroY")
plt.plot(x, gz, label="GyroZ")
plt.xlabel("Sample Index")
plt.ylabel("Angular Velocity (deg/s)")
plt.title("Gyroscope Data")
plt.legend()
plt.show()

# Save cleaned data
data.to_csv("data.csv", index=False)
print("Saved filtered data to data.csv")

