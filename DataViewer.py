import matplotlib.pyplot as plt
from os import listdir

time = []
altitude = []
v_vert = []
v_hor = []
delta_speed = []
pitch = []

files = listdir("report/")
for i, file in enumerate(files):
    print(f"{i+1}: {file}")

n = int(input("File index? "))
try:
    filename = files[n+1]
except:
    print("Invalid input!")

with open("report/" + filename, "r") as file:
    for line in file.readlines()[1:]:
        data = line.strip().split("|")

        time.append(float(data[0]))
        altitude.append(float(data[1]))
        v_vert.append(float(data[2]))
        v_hor.append(float(data[3]))
        delta_speed.append(float(data[4]))
        pitch.append(float(data[5]))

plt.figure(figsize=(10, 6))

plt.plot(time, altitude, label="Altitude")
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