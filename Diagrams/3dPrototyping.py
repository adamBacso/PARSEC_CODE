import matplotlib.pyplot as plt

x = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
y = [10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
z = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20]

fig = plt.figure()
temperature_plot = fig.add_subplot(326)
position_plot = fig.add_subplot(343, projection='3d')
intrnalTmperature_plot = fig.add_subplot(344)
navigation_plot = fig.add_subplot(325)
atmosphere_plot = fig.add_subplot(348)
acceleration_plot = fig.add_subplot(321)
gyroscope_plot = fig.add_subplot(323)
voc_plot = fig.add_subplot(347)

position_plot.title.set_text('Position')
temperature_plot.title.set_text('Temperature')
intrnalTmperature_plot.title.set_text('Internal Temperature')
navigation_plot.title.set_text('Navigation')
atmosphere_plot.title.set_text('Atmosphere')
acceleration_plot.title.set_text('Acceleration')
gyroscope_plot.title.set_text('Gyroscope')
voc_plot.title.set_text('VOC')

plt.show()