# Author: Djordje Kokot 
# Created: Sep 21, 2025
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data = pd.read_csv("output.csv")
data = data[(data != 0).all(axis=1)]
data = data[data["Recent Timestamp"] >= f"{pd.Timestamp('2025-08-20-')}"]
x = data.index
y1 = data["ADC1 Strain"].to_numpy()
y4 = data["ADC4 Strain"].to_numpy()
print(len(x),len(y1))
assert len(x) == len(y1), "index and row length must match"
import matplotlib.pyplot as plt
plt.plot(x, y1); plt.xlabel('index'); plt.ylabel(f'ADC1 Strain')
plt.show()
plt.plot(x, y4); plt.xlabel('index'); plt.ylabel(f'ADC1 Strain')
plt.show()
data.to_csv("data.csv", index=False)