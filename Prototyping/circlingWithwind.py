import turtle
import math

wn = turtle.Screen()
wn.bgcolor("white")
wn.title("Wind simulation")
kacat = turtle.Turtle()

def draw_path_with_wind(windSpeed, color="black"):
    turtle.color(color)
    resolution=100
    angle = 360/resolution
    radius = 100

    totalAngle=0
    turtle.speed(9)
    turtle.goto(0,0)
    for j in range(4):
        for i in range(resolution):
            turtle.forward(radius * math.pi * 2 / resolution + math.cos(totalAngle)*windSpeed)
            turtle.left(angle)
            totalAngle += angle * (math.pi/180)
    turtle.goto(0,0)


turtle.pendown()
draw_path_with_wind(2,"green")
draw_path_with_wind(1,"red")
draw_path_with_wind(-1,"blue")

turtle.done()