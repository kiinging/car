import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Setup serial port (try ttyACM0 first, or ttyAMA0 if it doesn't work)
ser = serial.Serial('/dev/ttyACM0', 115200)  # Or try '/dev/ttyAMA0'
ser.flushInput()

# Data storage
time_window = 10  # seconds
data_points = 100  # Number of points to display
xFiltRPS1 = np.zeros(data_points)
xFiltRPS2 = np.zeros(data_points)
time_array = np.linspace(0, time_window, data_points)

# Setup plot
plt.figure(figsize=(10, 5))
line1, = plt.plot(time_array, xFiltRPS1, label='Filtered RPS Method 1', color='blue')
line2, = plt.plot(time_array, xFiltRPS2, label='Filtered RPS Method 2', color='orange')
plt.xlim(0, time_window)
plt.ylim(-1, 1)  # Adjust based on expected RPS values
plt.xlabel('Time (s)')
plt.ylabel('Filtered RPM')
plt.title('Filtered RPM over Time')
plt.legend()
plt.grid()

def update(frame):
    global xFiltRPS1, xFiltRPS2
    
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        try:
            rps1, rps2 = map(float, line.split(','))
            # Shift data to the left
            xFiltRPS1 = np.roll(xFiltRPS1, -1)
            xFiltRPS2 = np.roll(xFiltRPS2, -1)
            # Update the last value
            xFiltRPS1[-1] = rps1
            xFiltRPS2[-1] = rps2
            
            line1.set_ydata(xFiltRPS1)
            line2.set_ydata(xFiltRPS2)
        except ValueError:
            pass  # Ignore any bad lines

    return line1, line2

# Animation
anim = FuncAnimation(plt.gcf(), update, interval=100, cache_frame_data=False)  # Assign the animation to a variable

plt.show()

# Close serial when done
ser.close()
