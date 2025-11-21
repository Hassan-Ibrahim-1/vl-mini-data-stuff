import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("out.csv")
data = data[(data != 0).any(axis=1)]
x = data.index

# Plot Accelerometer data
plt.figure(figsize=(12, 6))
plt.plot(x, data["Acc_X (m/s²)"], label="Acc_X")
plt.plot(x, data["Acc_Y (m/s²)"], label="Acc_Y")
plt.plot(x, data["Acc_Z (m/s²)"], label="Acc_Z")
plt.xlabel("Sample index")
plt.ylabel("Acceleration (m/s²)")
plt.title("Accelerometer Data")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# Plot Gyroscope data
plt.figure(figsize=(12, 6))
plt.plot(x, data["Gyro_X (deg/s)"], label="Gyro_X")
plt.plot(x, data["Gyro_Y (deg/s)"], label="Gyro_Y")
plt.plot(x, data["Gyro_Z (deg/s)"], label="Gyro_Z")
plt.xlabel("Sample index")
plt.ylabel("Angular velocity (deg/s)")
plt.title("Gyroscope Data")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
