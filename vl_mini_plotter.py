import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("out.csv")

data = data[(data != 0).any(axis=1)]

x = data.index

# Plot Accelerometer data
plt.figure(figsize=(12, 6))
plt.plot(x, data["AccX"], label="AccX")
plt.plot(x, data["AccY"], label="AccY")
plt.plot(x, data["AccZ"], label="AccZ")
plt.xlabel("Sample index")
plt.ylabel("Acceleration (g)")
plt.title("Accelerometer Data")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# Plot Gyroscope data
plt.figure(figsize=(12, 6))
plt.plot(x, data["GyroX"], label="GyroX")
plt.plot(x, data["GyroY"], label="GyroY")
plt.plot(x, data["GyroZ"], label="GyroZ")
plt.xlabel("Sample index")
plt.ylabel("Angular velocity (deg/s)")
plt.title("Gyroscope Data")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
