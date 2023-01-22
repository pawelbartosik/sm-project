import serial
from time import sleep
import time
import json
import matplotlib.pyplot as plt
import keyboard

plt.ion()

hSerial = serial.Serial('COM11', 115200, timeout=1, parity=serial.PARITY_NONE)
hSerial.write(b'2800')
sleep(0.5)
timestr = time.strftime("%Y%m%d-%H%M%S")
hFile = open("data_two_position_controller_%s.txt" % (timestr), "a")

hSerial.reset_input_buffer()
hSerial.flush()
temperature_samples = []
t = []
t_value=0
while True:
    text = hSerial.readline()
    temperature = 0
    sample = 0
    try:
        sample = json.loads(text)
        temperature = sample["temperature"]
    except ValueError:
        print("Bad JSON")
        print("%s\r\n" % {text})
        hSerial.flush()
        hSerial.reset_input_buffer()
    print(f'Aktualna temperatura:{temperature}')
    hFile.write("%.2f," % temperature)
    temperature_samples.append(temperature);
    t.append(t_value);
    t_value = t_value + 1
    plt.clf()
    plt.plot(t,temperature_samples, '.', markersize=5);
    plt.title("Wykres temperatury")
    plt.xlabel("Time (s)")
    plt.ylabel("Temperature (C)")
    plt.show()
    plt.pause(0.001)
    if keyboard.is_pressed("q"):
        break
hSerial.close()
hFile.close()
