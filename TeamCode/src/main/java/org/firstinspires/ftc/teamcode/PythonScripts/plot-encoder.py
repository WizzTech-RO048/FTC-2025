import matplotlib.pyplot as plt
import time

# Simulating data collection
encoder_values = []
time_stamps = []

for i in range(100):
    encoder_values.append(i * 10)  # Simulated encoder data
    time_stamps.append(time.time())
    time.sleep(0.1)

# Plot the data
plt.plot(time_stamps, encoder_values, marker='o', linestyle='-')
plt.xlabel("Time (s)")
plt.ylabel("Encoder Position")
plt.title("Encoder Position Over Time")
plt.grid(True)
plt.show()