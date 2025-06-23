import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load data
df = pd.read_csv('/home/mariam/multi_rob/src/grad/data/amcl_vs_gt.csv')

# Compute Euclidean error
df['pos_error'] = np.sqrt((df['amcl_x'] - df['gt_x'])**2 + (df['amcl_y'] - df['gt_y'])**2)

# Plot error over time
plt.plot(df['timestamp'], df['pos_error'], label='Position Error', color='red')
plt.xlabel('Time')
plt.ylabel('Error (meters)')
plt.title('AMCL vs Ground Truth Position Error')
plt.grid(True)
plt.legend()
plt.show()

# Optional: Print average error
print("Average positional error:", df['pos_error'].mean())
