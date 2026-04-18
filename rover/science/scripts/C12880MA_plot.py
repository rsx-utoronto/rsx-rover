# ===================================================
#       C12880MA Mini-spectrometer Arduino Read
#
#   This file reads and plots data from the C12880MA
#   mini-spectrometer from Hamamatsu
#
#


from science_serial_conector import Serial_Port
import numpy as np
import matplotlib.pyplot as plt

# import serial.tools.list_ports

# ports = serial.tools.list_ports.comports()
# for p in ports:
#     print(p.device, p.description)


PORT        = 'COM5'
BAUD        = 115200
NUM_PIXELS  = 288

# Make sure you change the IS_LINUX global in science_serial_connector
# as needed
spec_connection = Serial_Port(device_name= "C12880MA")

# ports = serial.tools.list_ports.comports()
# PORT = ports[2].device
print(PORT)
ser = spec_connection.open_device_port(port_name= PORT, baudrate= BAUD)

# CHECKED VALUES:
#  Pixel 79 = 530



summed = np.linspace(240, 850, 288)
draw_sum = False


plt.ion()
fig, ax = plt.subplots(figsize=(8, 4))
line, = ax.plot(np.linspace(340, 850, 288))
ax.set_ylim(0, 1000)
ax.set_xlabel("Pixel")
ax.set_ylabel("Intensity")

def find_max(arr):
    return np.max(arr)

# Spectrometer Calibration values 
# (provided by manufacturer for the part)
A0 = 3.036826093e2
B1 = 2.705020455e0
B2 = -1.120299836e-3
B3 = -7.562738106e-6
B4 = 8.112419810e-9
B5 = 7.087599537e-12
cl = lambda x: A0 +B1*x +B2*x**2 +B3*x**3 +B4*x**4 +B5*x**5
nm = [int(cl(i+1.)) for i in range(NUM_PIXELS)]

print("Listening on serial port... (Ctrl+C to quit)")

try:
    while True:
        # Use encoding "utf-8" on linux/macOS
        raw = ser.readline().decode('latin-1').strip()
        if not raw:
            continue
        try:
            data = np.array([int(x) for x in raw.split(',') if x.strip() != ''])
        except ValueError:
            print("WHAT")
            continue

        # Accumulate
        for i in range(min(len(data), NUM_PIXELS)):
            summed[i] += data[i]
        if draw_sum:
            summed_max = find_max(summed) / 1800 if find_max(summed) > 0 else 1
            y = summed / summed_max
        else:
            y = data[:NUM_PIXELS] / 2

        line.set_ydata(y)
        # line.set_xdata(np.linspace(340, 850, 288))
        line.set_xdata(nm)
        ax.set_xlim(300, 900)
        ax.set_ylim(0, max(y) * 1.1 + 1)

        plt.pause(0.001)

except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()