import matplotlib.pyplot as plt

# Inicialize listas vazias para cada coluna de dados
time = []
altitude = []
v_vert = []
v_hor = []
delta_speed = []
pitch = []

# Leia o arquivo de texto e preencha as listas de dados
with open('03-10-2023 19.35.txt', 'r') as file:
    for line in file.readlines()[1:]:
        data = line.strip().split('|')

        time.append(float(data[0]))
        altitude.append(float(data[1]))
        v_vert.append(float(data[2]))
        v_hor.append(float(data[3]))
        delta_speed.append(float(data[4]))
        pitch.append(float(data[5]))

plt.figure(figsize=(10, 6))

plt.plot(time, altitude, label='Altitude')
plt.plot(time, v_vert, label='Vel. Vert.')
plt.plot(time, v_hor, label='Vel. Hor.')
plt.plot(time, delta_speed, label='Delta Speed')
plt.plot(time, pitch, label='Pitch')

plt.xlabel('Tempo')
plt.ylabel('Valores')
plt.title('Valores X Tempo')

plt.legend()

plt.tight_layout()
plt.show()