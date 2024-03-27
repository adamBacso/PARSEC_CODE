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
import numpy as np
from os import system

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
        except (OSError, serial.SerialException):
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

time_data = [0]

internalTemp_data = [0]
latitude_data = [0]
longitude_data = [0]
courseToTarget_data = [0]
currentCourse_data = [0]
altitude_data = [0]
pressure_data = [0]
externalTemp_data = [0]
humidity_data = [0]
accelerationX_data = [0]
accelerationY_data = [0]
accelerationZ_data = [0]
gyroscopeX_data = [0]
gyroscopeY_data = [0]
gyroscopeZ_data = [0]
vocIndex_data = [0]

fig = plt.figure()
temperature_plot = fig.add_subplot(348)
#temperature_plot.title.set_text('Temperature')
temperature_plot.yaxis.tick_right()
temperature_plot.yaxis.set_label_position("right")
temperature_plot.set_xlabel('altitude (m)')
temperature_plot.set_ylabel('temperature (°C)')
internalTemp_plot = fig.add_subplot(344)
#internalTemp_plot.title.set_text('Internal Temperature')
internalTemp_plot.yaxis.tick_right()
internalTemp_plot.yaxis.set_label_position("right")
internalTemp_plot.set_xlabel('time (s)')
internalTemp_plot.set_ylabel('internal temperature (°C)')
position_plot = fig.add_subplot(343, projection='3d')
#position_plot.title.set_text('Position')
position_plot.set_xlabel('latitude (deg)')
position_plot.set_ylabel('longitude (deg)')
position_plot.set_zlabel('altitude (m)')
navigation_plot = fig.add_subplot(325)
#navigation_plot.title.set_text('Navigation')
navigation_plot.set_xlabel('time (s)')
navigation_plot.set_ylabel('degrees cardinal (deg)')
pressure_plot = fig.add_subplot(3,4,12)
pressure_plot.yaxis.tick_right()
pressure_plot.yaxis.set_label_position("right")
#pressure_plot.title.set_text('Pressure')
pressure_plot.set_xlabel('time (s)')
pressure_plot.set_ylabel('pressure (hPa)')
humidity_plot = fig.add_subplot(3,4,11)
#humidity_plot.title.set_text('Humidity')
humidity_plot.set_xlabel('altitude (m)')
humidity_plot.set_ylabel('humidity (%)')
acceleration_plot = fig.add_subplot(321)
#acceleration_plot.title.set_text('Acceleration')
acceleration_plot.set_xlabel('time (s)')
acceleration_plot.set_ylabel('acceleration (m/s^2)')
gyroscope_plot = fig.add_subplot(323)
#gyroscope_plot.title.set_text('Gyroscope')
gyroscope_plot.set_xlabel('time (s)')
gyroscope_plot.set_ylabel('gyroscope (rad/s)')
voc_plot = fig.add_subplot(347)
#voc_plot.title.set_text('VOC')
voc_plot.set_xlabel('altitude (m)')
voc_plot.set_ylabel('voc index')

def read_and_process_data(fromHex = False):
    """ Reads data from serial port and appends data to each appropriate list

        :param fromHex:
            Whether or not the data is in hex
    """"""
    try:
        line = kacat.readline().decode('utf-8').strip() # TODO: save data to file
    except SerialException:
        print("Serial device disconnected. Exiting...")
        return
    if fromHex:
        if (line.startswith("radio_rx ")):
            line = line[len("radio_rx "):].strip()
            line = bytes.fromhex(line).decode('ascii')
    telemetryData = line.split(',')
    """
    time_data.append(float(telemetryData[1]))
    internalTemp_data.append(float(telemetryData[2]))
    latitude_data.append(float(telemetryData[3]))
    longitude_data.append(float(telemetryData[4]))
    courseToTarget_data.append(float(telemetryData[5]))
    currentCourse_data.append(float(telemetryData[7]))
    altitude_data.append(float(telemetryData[8]))
    pressure_data.append(float(telemetryData[9]))
    externalTemp_data.append(float(telemetryData[10]))
    humidity_data.append(float(telemetryData[11]))
    accelerationX_data.append(float(telemetryData[12]))
    accelerationY_data.append(float(telemetryData[13]))
    accelerationZ_data.append(float(telemetryData[14]))
    gyroscopeX_data.append(float(telemetryData[15]))
    gyroscopeY_data.append(float(telemetryData[16]))
    gyroscopeZ_data.append(float(telemetryData[17]))
    vocIndex_data.append(float(telemetryData[18]))

    print("Telemetry ID: ", telemetryData[0])
    print("Time: ",time_data[-1])
    # POSITION
    print("Latitude: ",latitude_data[-1])
    print("Longitude: ",longitude_data[-1])
    print("Altitude: ",altitude_data[-1])
    # NAVIGATION
    print("Course to target: ",courseToTarget_data[-1])
    print("Current course: ",currentCourse_data[-1])
    # ATMOSPHERE
    print("Pressure: ",pressure_data[-1])
    print("Humidity: ",humidity_data[-1])
    # Temperature
    print("External temperature: ",externalTemp_data[-1])
    print("Internal temperature: ",internalTemp_data[-1])
    # ACCELERATION
    print("Acceleration X: ",accelerationX_data[-1])
    print("Acceleration Y: ",accelerationY_data[-1])
    print("Acceleration Z: ",accelerationZ_data[-1])
    # GYROSCOPE
    print("Gyroscope X: ",gyroscopeX_data[-1])
    print("Gyroscope Y: ",gyroscopeY_data[-1])
    print("Gyroscope Z: ",gyroscopeZ_data[-1])
    # VOC
    print("VOC index: ",vocIndex_data[-1])
    print("\n~~~~~~\n")

def update_plot(frame):
    read_and_process_data(True)

    # TEMPERATURE
    temperature_plot.clear()
    temperature_plot.plot(altitude_data, externalTemp_data)
    
    internalTemp_plot.clear()
    internalTemp_plot.plot(time_data, internalTemp_data)

    # ACCELERATION
    acceleration_plot.clear()
    acceleration_plot.plot(time_data, accelerationX_data, label="acceleration X")
    acceleration_plot.plot(time_data, accelerationY_data, label="acceleration Y")
    acceleration_plot.plot(time_data, accelerationZ_data, label="acceleration Z")
    acceleration_plot.legend()

    # GYROSCOPE
    gyroscope_plot.clear()
    gyroscope_plot.plot(time_data, gyroscopeX_data, label="gyroscope X")
    gyroscope_plot.plot(time_data, gyroscopeY_data, label="gyroscope Y")
    gyroscope_plot.plot(time_data, gyroscopeZ_data, label="gyroscope Z")
    gyroscope_plot.legend()

    # POSITION
    position_plot.clear()
    position_plot.plot(longitude_data, latitude_data, altitude_data, label="flight path")
    position_plot.scatter(targetLon, targetLat, altitude_data[0], c="red", label="target")
    position_plot.legend()

    # NAVIGATION
    navigation_plot.clear()
    navigation_plot.plot(time_data, courseToTarget_data, label="course to target")
    navigation_plot.plot(time_data, currentCourse_data, label="current course")
    navigation_plot.legend()

    # ATMOSPHERE
    pressure_plot.clear()
    pressure_plot.plot(time_data, pressure_data)

    humidity_plot.clear()
    humidity_plot.plot(altitude_data, humidity_data)
    # VOC
    voc_plot.clear()
    voc_plot.plot(altitude_data, vocIndex_data, label="voc index")

ani = FuncAnimation(fig, update_plot, interval=10, save_count=100)
plt.show()
read_and_process_data()