import matplotlib.pyplot as plt
import numpy as np

packetID = []
time = []
internalTemperature = []
gpsAge = []
gpsLatitude = []
gpsLongitude = []
gpsAltitude = []
barometricAltitude = []
bmeTemperature = []
humidity = []
accelerationX = []
accelerationY = []
accelerationZ = []
gyroscopeX = []
gyroscopeY = []
gyroscopeZ = []
mpuTemperature = []


def readFile(fileName):
    f = open(fileName, "rt")
    for line in f:
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
        gpsAltitude.append(lineArray[index])
        index += 1
        barometricAltitude.append(lineArray[index])
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

def draw_plot(y, x = [], formatting = "o"):
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

        plt.plot(x,y,formatting)

def show_plot(title = ""):
    plt.title(title)
    plt.show()

readFile("Diagrams/test.txt")
draw_plot(internalTemperature,time)
show_plot("Test Diagram")