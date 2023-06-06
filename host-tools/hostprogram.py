from tkinter import *
from tkinter import ttk

# Sensor class
class sensor:
    width = 15
    height = 100

    def __init__(self, canv: Canvas, type: str, axis: str, x: int, y: int) -> None:
        self.canvas = canv
        self.type = type
        self.outline = 'white'
        self.fill = 'black'

        self.input = input
        
        self.x = x
        self.y = y
        self.fillX = self.x + 7
        self.fillY = (self.y + self.height/2)
        self.fillWidth = 1
        self.fillHeight = 0
        self.value = 0.0

        # Range of the sensor
        if type == 'gyro':
            self.range = 2000.0
        elif type == 'accell':
            self.range = 16.0
        else:
            self.range = 8.1

        self.outlineid = self.canvas.create_rectangle(
            self.x,
            self.y,
            self.x + self.width,
            self.y + self.height,
            fill=self.fill,
            outline=self.outline,
            tags=(type + axis,  'outline')
        )

        self.fillBar = self.canvas.create_rectangle(
            self.fillX,
            self.fillY,
            self.fillX + self.fillWidth,
            self.fillY + self.fillHeight,
            fill='white',
            outline='white',
            tags=(type + axis,  'fillBar')
        )
        self.middleline = self.canvas.create_line(
            self.x + 3,
            self.y + self.height/2,
            self.x + 12,
            self.y + self.height/2,
            fill=self.outline,
            tags=(type + axis,  'line')
        )

        self.label = self.canvas.create_text(
            (self.x + self.width/2),
            self.y - 15,
            fill=self.outline, 
            font='TkFixedFont',
            text=axis.capitalize() 
        )

        self.valueLabel = self.canvas.create_text(
            (self.x + self.width/2),
            self.y + 120,
            fill=self.outline, 
            font='TkFixedFont',
            text=str(self.value)
        )
    
    def updateValue(self, value: float) -> None:
        # Bar fills up to the edge of the outline, with 3 px spacing, proportionately to the angle
        self.value = value
        self.fillHeight = (((self.height/2) - 3)/self.range) * self.value
        self.canvas.coords(self.fillBar, self.fillX, self.fillY, self.fillX + self.fillWidth, self.fillY + self.fillHeight)
        self.canvas.itemconfigure(self.valueLabel, text=str(self.value))


# Servo Class
class servo:
    width = 100
    height = 15
    range = 45

    def __init__(self, canv: Canvas, servonum: int, x: int, y: int, input: Entry) -> None:
        self.canvas = canv
        self.servonum = servonum
        self.outline = 'white'
        self.fill = 'black'

        self.input = input
        
        self.x = x
        self.y = y
        self.fillX = (self.x + self.width/2)
        self.fillY = self.y + 7
        self.fillWidth = 0
        self.fillHeight = 1
        self.angle = 0.0

        self.outlineid = self.canvas.create_rectangle(
            self.x,
            self.y,
            self.x + self.width,
            self.y + self.height,
            fill=self.fill,
            outline=self.outline,
            tags=('servo' + str(servonum), 'outline')
        )

        self.fillBar = self.canvas.create_rectangle(
            self.fillX,
            self.fillY,
            self.fillX + self.fillWidth,
            self.fillY + self.fillHeight,
            fill='white',
            outline='white',
            tags=('servo' + str(servonum), 'fillBar')
        )
        self.middleline = self.canvas.create_line(
            (self.x + self.width/2),
            self.y + 3,
            (self.x + self.width/2),
            self.y + self.height - 3,
            fill=self.outline,
            tags=('servo' + str(servonum), 'line')
        )

        self.label = self.canvas.create_text(
            (self.x + self.width/2),
            self.y - 15,
            fill=self.outline, 
            font='TkFixedFont',
            text='Servo ' + str(self.servonum)
        )

        self.angleLabel = self.canvas.create_text(
            (self.x + self.width/2),
            self.y + 30,
            fill=self.outline, 
            font='TkFixedFont',
            text='Angle: ' + str(self.angle)
        )
    
    def updateAngle(self, angle: float) -> None:
        # Bar fills up to the edge of the outline, with 3 px spacing, proportionately to the angle
        self.angle = angle
        self.fillWidth = (((self.width/2) - 3)/self.range) * self.angle
        self.canvas.coords(self.fillBar, self.fillX, self.fillY, self.fillX + self.fillWidth, self.fillY + self.fillHeight)
        self.canvas.itemconfigure(self.angleLabel, text='Angle: ' + str(self.angle))
    
    def updateFromEntry(self) -> None:
        try:
            ang = self.input.get()
            self.updateAngle(float(ang))
        except:
            pass


# Window
window = Tk()
window.title('Host Tools')
window.resizable(False, False)

# Window frame
root = Frame(window)
root.grid(column=0, row=0, padx=3, pady=3)

# Canvas
canvas = Canvas(root, background='black', width=480, height=320)
canvas.grid(column=0, row=0, columnspan=4, sticky=(N, W, E, S))

# Servos and their buttons
input1text = StringVar()
input1 = ttk.Entry(root, textvariable=input1text, width=7)
servobar1 = servo(canvas, 1, 25, 30, input1)
button1 = Button(root, text='Servo 1', command=servobar1.updateFromEntry)
input1.grid(column=0, row=1)
button1.grid(column=0, row=2)

input2text = StringVar()
input2 = ttk.Entry(root, textvariable=input2text, width=7)
servobar2 = servo(canvas, 2, 135, 30, input2)
button1 = Button(root, text='Servo 2', command=servobar2.updateFromEntry)
input2.grid(column=1, row=1)
button1.grid(column=1, row=2)

input3text = StringVar()
input3 = ttk.Entry(root, textvariable=input3text, width=7)
servobar3 = servo(canvas, 3, 245, 30, input3)
button1 = Button(root, text='Servo 3', command=servobar3.updateFromEntry)
input3.grid(column=2, row=1)
button1.grid(column=2, row=2)

input4text = StringVar()
input4 = ttk.Entry(root, textvariable=input4text, width=7)
servobar4 = servo(canvas, 4, 355, 30, input4)
button1 = Button(root, text='Servo 4', command=servobar4.updateFromEntry)
input4.grid(column=3, row=1)
button1.grid(column=3, row=2)

# Sensors
gyrolabel = canvas.create_text(87, 110, fill='white', text='Gyro')
gyroX = sensor(canvas, 'gyro', 'x', 50, 150)
gyroY = sensor(canvas, 'gyro', 'y', 80, 150)
gyroZ = sensor(canvas, 'gyro', 'z', 110, 150)

accellabel = canvas.create_text(239, 110, fill='white', text='Accel')
accelX = sensor(canvas, 'accel', 'x', 202, 150)
accelY = sensor(canvas, 'accel', 'y', 232, 150)
accelZ = sensor(canvas, 'accel', 'z', 262, 150)

maglabel = canvas.create_text(392, 110, fill='white', text='Mag')
magX = sensor(canvas, 'mag', 'x', 355, 150)
magY = sensor(canvas, 'mag', 'y', 385, 150)
magZ = sensor(canvas, 'mag', 'z', 415, 150)

def update_sensors():
    pass

def frame():
    canvas.after(20, frame)
    update_sensors()

root.mainloop()
