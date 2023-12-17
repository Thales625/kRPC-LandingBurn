import matplotlib.pyplot as plt
from os import listdir

time = []
altitude = []
dist = []
v_vert = []
v_hor = []
delta_speed = []
pitch = []

files = listdir("report/")
for i, file in enumerate(files):
    print(f"{i+1}: {file}")

n = int(input("File index: "))
try:
    filename = files[n if n == -1 else n-1]
    print("Opening file", filename)
except:
    print("Invalid input!")
    exit()

with open("report/" + filename, "r") as file:
    for line in file.readlines()[1:]:
        data = line.strip().split("|")

        time.append(float(data[0]))
        altitude.append(float(data[1]))
        dist.append(float(data[2]))
        v_vert.append(float(data[3]))
        v_hor.append(float(data[4]))
        delta_speed.append(float(data[5]))
        pitch.append(float(data[6]))

plt.figure(figsize=(10, 7))

plt.plot(time, altitude, label="Altitude")
plt.plot(time, dist, label="Distance")
plt.plot(time, v_vert, label="Vel. Vert.")
plt.plot(time, v_hor, label="Vel. Hor.")
plt.plot(time, delta_speed, label="Delta Speed")
plt.plot(time, pitch, label="Pitch")

plt.xlabel("Time")
plt.ylabel("Values")
plt.title("Values X Time")

plt.legend()

plt.tight_layout()
plt.show()