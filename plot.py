import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def read_sensor_data(file_path):
    data = pd.read_csv(file_path)
    expected_columns = ["Time", "Sensor1", "Sensor2"]
    if not all(col in data.columns for col in expected_columns):
        return None
    
    data["Time"] = pd.to_numeric(data["Time"], errors="coerce")
    data["Sensor1"] = pd.to_numeric(data["Sensor1"], errors="coerce")
    data["Sensor2"] = pd.to_numeric(data["Sensor2"], errors="coerce")
    
    return data.dropna()

def plot_sensor_data(data):
    if data is None or data.empty:
        return
    
    time_data = data['Time'].to_numpy()
    sensor1_data = data['Sensor1'].to_numpy()
    sensor2_data = data['Sensor2'].to_numpy()
    
    plt.figure(figsize=(12, 6))
    plt.plot(time_data, sensor1_data, label='1st sensor', color='green', linewidth=1.5)
    plt.plot(time_data, sensor2_data, label='2nd sensor', color='blue', linestyle='dashed', linewidth=1.5)
    
    plt.xlabel('Time in microseconds)')
    plt.ylabel(' Sensor wave values')
    plt.title('Sine sensor plot snapshot 10 seconds')
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.show()

def main():
    file_path = "sensor_data.csv"
    data = read_sensor_data(file_path)
    if data is not None:
        plot_sensor_data(data)

if __name__ == "__main__":
    main()
