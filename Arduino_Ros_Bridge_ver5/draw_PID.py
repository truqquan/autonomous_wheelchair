import pandas as pd
import matplotlib.pyplot as plt

# Load and clean data
df = pd.read_csv('pid_data.csv')
df = df.dropna()

# Convert data types
df['TargetLV'] = pd.to_numeric(df['TargetLV'], errors='coerce')
df['CurrentLV'] = pd.to_numeric(df['CurrentLV'], errors='coerce')
df['ControlSignal1'] = pd.to_numeric(df['ControlSignal1'], errors='coerce')
df['Timestamp'] = pd.to_datetime(df['Timestamp'], format='%H:%M:%S.%f', errors='coerce')

# Drop rows with any invalid values
df = df.dropna(subset=['TargetLV', 'CurrentLV', 'ControlSignal1', 'Timestamp'])

# Calculate relative time in seconds
df['Time_seconds'] = (df['Timestamp'] - df['Timestamp'].iloc[0]).dt.total_seconds()

# Filter data from 50 to 60 seconds
df_zoomed = df[(df['Time_seconds'] >= 13.0) & (df['Time_seconds'] <= 20.0)]

# Plot with millisecond resolution
plt.figure(figsize=(12, 6))
plt.plot(df_zoomed['Time_seconds'].to_numpy(), df_zoomed['TargetLV'].to_numpy(), label='TargetLV', color='red')
plt.plot(df_zoomed['Time_seconds'].to_numpy(), df_zoomed['CurrentLV'].to_numpy(), label='CurrentLV', color='blue')
plt.plot(df_zoomed['Time_seconds'].to_numpy(), df_zoomed['ControlSignal1'].to_numpy(), label='u1', color='green')

plt.xlabel('Time (seconds, with millisecond resolution)')
plt.ylabel('Values')
plt.title('PID Data (13s to 20s)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
