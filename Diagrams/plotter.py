import matplotlib.pyplot as plt
import numpy as np

packetID = []
time = []
internalTemperature = []
gpsAge = []
gpsLatitude = []
gpsLongitude = []
courseToTarget = []
distanceToTarget = []
currentCourse = []
gpsAltitude = []
barometricAltitude = []
verticalSpeed = []
bmeTemperature = []
humidity = []
accelerationX = []
accelerationY = []
accelerationZ = []
gyroscopeX = []
gyroscopeY = []
gyroscopeZ = []
mpuTemperature = []

telemetryPreamble = "radio_rx "

def readFile(fileName):
    f = open(fileName, "rt")

    for line in f:

        if line.startswith(telemetryPreamble):
            line = line[len(line)]
            line = bytes.fromhex(line).decode('utf-8')

        lineArray = line.split(",")
        index = 0
        packetID.append(lineArray[index])
        index += 1
        time.append(lineArray[index])
        index += 1
        internalTemperature.append(lineArray[index])
        index += 1
        gpsAge.append(lineArray[index])
        index += 1
        gpsLatitude.append(lineArray[index])
        index += 1
        gpsLongitude.append(lineArray[index])
        index += 1
        courseToTarget.append(lineArray[index])
        index += 1
        distanceToTarget.append(lineArray[index])
        index += 1
        currentCourse.append(lineArray[index])
        index += 1
        gpsAltitude.append(lineArray[index])
        index += 1
        barometricAltitude.append(lineArray[index])
        index += 1
        verticalSpeed.append(lineArray[index])
        index += 1
        bmeTemperature.append(lineArray[index])
        index += 1
        humidity.append(lineArray[index])
        index += 1
        accelerationX.append(lineArray[index])
        index += 1
        accelerationY.append(lineArray[index])
        index += 1
        accelerationZ.append(lineArray[index])
        index += 1
        gyroscopeX.append(lineArray[index])
        index += 1
        gyroscopeY.append(lineArray[index])
        index += 1
        gyroscopeZ.append(lineArray[index])
        index += 1
        mpuTemperature.append(lineArray[index])
        index += 1
    f.close()

def draw_plot(y, x = [], formatting = "o-", label = ""):
    if x == []:
        for i in range(len(y)):
            x.append(i)

    if len(x) == len(y):
        i = 0
        while i<len(y):
            try:
                y[i] = float(y[i])
                i+=1
            except ValueError:
                del y[i]
                del x[i]
        
        i = 0
        while i<len(x):
            try:
                x[i] = float(x[i])
                i+=1
            except ValueError:
                del y[i]
                del x[i]

        plt.plot(x,y,formatting, label=label, markersize=3)

def show_plot(title = ""):
    plt.legend()
    plt.title(title)
    plt.show()
readFile("Logging/on-board data/flight3.txt")

draw_plot(barometricAltitude,time,label="Altitude (BME)")
show_plot("Altitude - time")

draw_plot(verticalSpeed,time,label="Vertical Speed")
show_plot("VSpeed - time")

draw_plot(accelerationX,time, label="Acceleration X")
draw_plot(accelerationY,time, label="Acceleration Y")
draw_plot(accelerationZ,time, label="Acceleration Z")
show_plot("Acceleration - time graph")

draw_plot(gyroscopeX,time, label="Rotation X")
draw_plot(gyroscopeY,time, label="Rotation Y")
draw_plot(gyroscopeZ,time, label="Rotation Z")
show_plot("Rotation - time graph")