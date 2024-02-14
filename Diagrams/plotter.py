import matplotlib.pyplot as plt
import numpy as np

# Initialize a dictionary to hold all telemetry data
telemetryData = {}

telemetryPreamble = "radio_rx "

def readFile(fileName):
    with open(fileName, "rt") as f:
        # Read the first line to get keys (column names)
        keys = f.readline().strip().split(",")
        # Initialize lists for each key in the dictionary
        for key in keys:
            telemetryData[key] = []

        for line in f:
            if line.startswith(telemetryPreamble):
                line = line[len(telemetryPreamble):].strip()
                line = bytes.fromhex(line).decode('utf-8')

            lineArray = line.strip().split(",")
            # Check if the lineArray length matches keys length to avoid index errors
            if len(lineArray) == len(keys):
                for key, value in zip(keys, lineArray):
                    telemetryData[key].append(value)

def draw_plot(yLabel, xLabel = 'missionTime', formatting = "o-", label = ""):
    y = telemetryData[yLabel]
    x = telemetryData[xLabel]
    if not x:
        x = list(range(len(y)))

    # Convert x and y to floats, discarding unconvertible values
    x, y = zip(*[(float(xi), float(yi)) for xi, yi in zip(x, y) if xi.replace('.','',1).isdigit() and yi.replace('.','',1).isdigit()])

    plt.plot(x, y, formatting, label=label, markersize=3)

def show_plot(title = ""):
    plt.legend()
    plt.title(title)
    plt.show()

def spacial_plot(xData='latitude', yData='longitude', zData='barometricAltitude', xLabel='Latitude', yLabel="Longitude", zLabel='Altitude (BME)', xTarget=47.4979, yTarget=19.0402):
    x = telemetryData[xData]
    y = telemetryData[yData]
    z = telemetryData[zData]

    fig = plt.figure()
    splot = fig.add_subplot(projection='3d')
    x, y, z = zip(*[(float(xi), float(yi), float(zi)) for xi, yi, zi in zip(x, y, z) if xi.replace('.', '', 1).isdigit() and yi.replace('.', '', 1).isdigit() and zi.replace('.', '', 1).isdigit()])

    splot.scatter(x,y,z,marker="o")

    splot.set_xlabel(xLabel)
    splot.set_ylabel(yLabel)
    splot.set_zlabel(zLabel)

    plt.show()

# Example usage
readFile("Logging/on-board data/flight3.txt")

draw_plot('barometricAltitude', label="Altitude (BME)")
show_plot("Altitude - time")

draw_plot('verticalSpeed', label="Vertical Speed")
show_plot("VSpeed - time")

draw_plot('accelerationX', label="Acceleration X")
draw_plot('accelerationY', label="Acceleration Y")
draw_plot('accelerationZ', label="Acceleration Z")
show_plot("Acceleration - time graph")

draw_plot('gyroscopeX', label="Rotation X")
draw_plot('gyroscopeY', label="Rotation Y")
draw_plot('gyroscopeZ', label="Rotation Z")
show_plot("Rotation - time graph")

draw_plot('courseToTarget', label="Course to target")
draw_plot('currentCourse', label="Current course")
show_plot("Current and desired course")

spacial_plot()