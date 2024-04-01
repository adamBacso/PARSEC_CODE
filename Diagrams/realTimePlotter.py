import pip
def install(package):
    """ Installs dependencies

        :param package:
            The name of the package to install
        :returns:
            None
    """
    pip.main(['install', package])
try:
    import serial
except ImportError:
    install('pyserial')
    import serial

try:
    import matplotlib.pyplot as plt
except ImportError:
    install('matplotlib')
    import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv
import numpy as np
from os import system
import os.path

def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    ports = ['COM%s' % (i + 1) for i in range(256)]

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError):
            pass
    return result

availablePorts = serial_ports()
print(availablePorts)
index = int(input())
S_PORT = availablePorts[index]
S_BAUD = 9600


kacat = serial.Serial(S_PORT, S_BAUD)

targetLat = 0
targetLon = 0

time_data = []

latitude_data = []
longitude_data = []
altitude_data = []
pressure_data = []
tempererature_data = []
vocIndex_data = []

data = [time_data,latitude_data,longitude_data,altitude_data,pressure_data,tempererature_data,vocIndex_data]

receiving = True
def read_and_process_data():
    """ Reads data from serial port and appends data to each appropriate list

        :param fromHex:
            Whether or not the data is in hex
    """
    try:
        line = kacat.readline().decode('utf-8').strip() # TODO: save data to file
    except serial.serialutil.SerialException:
        print("Serial device disconnected. Exiting...")
        global receiving
        receiving = False
        return
    if (line.startswith("->")):
        line = line[2:]
        telemetryData = line.split(',')
        
        for i in range(len(data)-1):
            data[i].append(telemetryData[i])
        print(line)

def save_to_csv():
    fileNameBase = "incoming_data"
    fileExtension = ".csv"
    fileID = 1

    fileName = fileNameBase + str(fileID) + fileExtension # TODO: fix writing to csv
    while (os.path.exists(fileName)):
        fileID = fileID + 1
        fileName = fileNameBase + str(fileID) + fileExtension
    
    with open(fileName, 'w', newline='') as file:
        writer = csv.writer(file)
        for i in range(len(time_data)-1):
            writer.writerow([time_data[i],latitude_data[i],longitude_data[i],altitude_data[i],pressure_data[i],tempererature_data[i],vocIndex_data[i]])

while receiving:
    read_and_process_data()

save_to_csv()