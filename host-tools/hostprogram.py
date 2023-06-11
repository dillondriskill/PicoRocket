from tkinter import *
from tkinter import messagebox
from tkinter import ttk

import serial
import math
import time
import os


# Sensor class
class sensor:
    width = 15
    height = 100

    def __init__(self, canv: Canvas, type: str, axis: str, x: int, y: int) -> None:
        self.canvas = canv
        self.type = type
        self.outline = "white"
        self.fill = "black"

        self.x = x
        self.y = y
        self.fillX = self.x + 7
        self.fillY = self.y + self.height / 2
        self.fillWidth = 1
        self.fillHeight = 0
        self.value = 0.0

        # Range of the sensor
        if type == "gyro":
            self.range = 2000.0
        elif type == "accell":
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
            tags=(type + axis, "outline"),
        )

        self.fillBar = self.canvas.create_rectangle(
            self.fillX,
            self.fillY,
            self.fillX + self.fillWidth,
            self.fillY + self.fillHeight,
            fill="white",
            outline="white",
            tags=(type + axis, "fillBar"),
        )
        self.middleline = self.canvas.create_line(
            self.x + 3,
            self.y + self.height / 2,
            self.x + 12,
            self.y + self.height / 2,
            fill=self.outline,
            tags=(type + axis, "line"),
        )

        self.label = self.canvas.create_text(
            (self.x + self.width / 2),
            self.y - 15,
            fill=self.outline,
            font="TkFixedFont",
            text=axis.capitalize(),
        )

        self.valueLabel = self.canvas.create_text(
            (self.x + self.width / 2),
            self.y + 120,
            fill=self.outline,
            font="TkFixedFont",
            text=str(self.value),
        )

    def updateValue(self, value: float) -> None:
        # Bar fills up to the edge of the outline, with 3 px spacing, proportionately to the angle
        self.value = value
        self.fillHeight = (((self.height / 2) - 3) / self.range) * self.value
        self.canvas.coords(
            self.fillBar,
            self.fillX,
            self.fillY,
            self.fillX + self.fillWidth,
            self.fillY + self.fillHeight,
        )
        self.canvas.itemconfigure(self.valueLabel, text=str(self.value))


# Servo Class
class servo:
    width = 100
    height = 15
    range = 45

    def __init__(
        self, canv: Canvas, servonum: int, x: int, y: int, input: Entry
    ) -> None:
        self.canvas = canv
        self.servonum = servonum
        self.outline = "white"
        self.fill = "black"

        self.input = input

        self.x = x
        self.y = y
        self.fillX = self.x + self.width / 2
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
            tags=("servo" + str(servonum), "outline"),
        )

        self.fillBar = self.canvas.create_rectangle(
            self.fillX,
            self.fillY,
            self.fillX + self.fillWidth,
            self.fillY + self.fillHeight,
            fill="white",
            outline="white",
            tags=("servo" + str(servonum), "fillBar"),
        )
        self.middleline = self.canvas.create_line(
            (self.x + self.width / 2),
            self.y + 3,
            (self.x + self.width / 2),
            self.y + self.height - 3,
            fill=self.outline,
            tags=("servo" + str(servonum), "line"),
        )

        self.label = self.canvas.create_text(
            (self.x + self.width / 2),
            self.y - 15,
            fill=self.outline,
            font="TkFixedFont",
            text="Servo " + str(self.servonum),
        )

        self.angleLabel = self.canvas.create_text(
            (self.x + self.width / 2),
            self.y + 30,
            fill=self.outline,
            font="TkFixedFont",
            text="Angle: " + str(self.angle),
        )

    def updateAngle(self, angle: float) -> None:
        # Bar fills up to the edge of the outline, with 3 px spacing, proportionately to the angle
        self.angle = angle
        self.fillWidth = (((self.width / 2) - 3) / self.range) * self.angle
        self.canvas.coords(
            self.fillBar,
            self.fillX,
            self.fillY,
            self.fillX + self.fillWidth,
            self.fillY + self.fillHeight,
        )
        self.canvas.itemconfigure(self.angleLabel, text="Angle: " + str(self.angle))

    def updateFromEntry(self) -> None:
        try:
            ang = self.input.get()
            self.updateAngle(float(ang))
        except:
            pass


class handler:
    def __init__(
        self,
        servo1: servo,
        servo2: servo,
        servo3: servo,
        servo4: servo,
        ax: sensor,
        ay: sensor,
        az: sensor,
        mx: sensor,
        my: sensor,
        mz: sensor,
        gx: sensor,
        gy: sensor,
        gz: sensor,
        port: Entry,
        file: Entry,
        canvas: Canvas,
        root: ttk.Frame,
    ) -> None:
        self.servo1 = servo1
        self.servo2 = servo2
        self.servo3 = servo3
        self.servo4 = servo4
        self.ax = ax
        self.ay = ay
        self.az = az
        self.mx = mx
        self.my = my
        self.mz = mz
        self.gx = gx
        self.gy = gy
        self.gz = gz
        self.port = port
        self.file = file
        self.canvas = canvas
        self.root = root

        self.device = serial.Serial()
        self.frameid = ""

        # Test Frame
        # self.testroot = None
        # self.testframe = None

    def update_servos(self):
        self.servo1.updateFromEntry()
        self.servo2.updateFromEntry()
        self.servo3.updateFromEntry()
        self.servo4.updateFromEntry()

        self.device.write(b"\x01")
        self.device.read()

        self.device.write(int(self.servo1.angle).to_bytes(1, "little", signed=True))
        self.device.read()

        self.device.write(int(self.servo2.angle).to_bytes(1, "little", signed=True))
        self.device.read()

        self.device.write(int(self.servo2.angle).to_bytes(1, "little", signed=True))
        self.device.read()

        self.device.write(int(self.servo2.angle).to_bytes(1, "little", signed=True))
        self.device.read()

    def update_sensors(self):
        self.device.write(b'\x00')
        self.device.read()

        for i in range(0, 1):
            char = int.from_bytes(self.device.read())
            self.labels[i]['text'] = ("0x{0:2X}".format(i+0x26) + ": " + "{:03d} {:02X} {:08b}".format(char, char, char))
            self.device.write(b'\x00')
        self.device.write(b'\x00')

    def connect(self):
        self.device = serial.Serial(self.port.get(), baudrate=115200)
        self.frameid = self.canvas.after(20, self.frame)
        print("Connected")

        self.testroot = Tk()
        self.testroot.title("Gyro Sensor Outputs")
        self.testroot.geometry("180x500")
        self.testframe = ttk.Frame(self.testroot)
        self.testframe.grid(column=0, row=0, sticky=(N, S, E, W))

        self.labels = []
        for i in range(0, 1):
            item = ttk.Label(self.testframe, text='0xFF', font='TkFixedFont')
            item.grid(column=0, row=i)
            self.labels.append(item)

    def upload(self):
        self.root.configure(cursor="watch")
        try:
            self.canvas.after_cancel(self.frameid)
            for i in range(0, 10):
                self.device.write(b"\x04")
        except:
            pass
        messagebox.showinfo("Host Tools", "Press OK to upload!")
        time.sleep(2)
        for i in range(0, 10):
            os.system("cp " + self.file.get() + " /Volumes/RPI-RP2/rocket.uf2")
        time.sleep(1)
        self.root.configure(cursor="")
        print("Uploaded")

    def frame(self):
        
        #self.update_servos()
        self.update_sensors()
        self.frameid = self.canvas.after(20, self.frame)


def main():
    # Window
    window = Tk()
    window.title("Host Tools")
    window.resizable(False, False)

    # Window frame
    root = ttk.Frame(window)
    root.grid(column=0, row=0, padx=3, pady=3)

    # Canvas
    canvas = Canvas(root, background="black", width=480, height=320)
    canvas.grid(column=0, row=0, columnspan=4, sticky=(N, W, E, S))

    # Servo inputs
    input1text = DoubleVar(value=0.0)
    input1 = ttk.Entry(root, textvariable=input1text, width=7)
    servobar1 = servo(canvas, 1, 25, 30, input1)
    input1.grid(column=0, row=1)

    input2text = DoubleVar(value=0.0)
    input2 = ttk.Entry(root, textvariable=input2text, width=7)
    servobar2 = servo(canvas, 2, 135, 30, input2)
    input2.grid(column=1, row=1)

    input3text = DoubleVar(value=0.0)
    input3 = ttk.Entry(root, textvariable=input3text, width=7)
    servobar3 = servo(canvas, 3, 245, 30, input3)
    input3.grid(column=2, row=1)

    input4text = DoubleVar(value=0.0)
    input4 = ttk.Entry(root, textvariable=input4text, width=7)
    servobar4 = servo(canvas, 4, 355, 30, input4)
    input4.grid(column=3, row=1)

    # Sensors
    gyrolabel = canvas.create_text(87, 110, fill="white", text="Gyro")
    gyroX = sensor(canvas, "gyro", "x", 50, 150)
    gyroY = sensor(canvas, "gyro", "y", 80, 150)
    gyroZ = sensor(canvas, "gyro", "z", 110, 150)

    accellabel = canvas.create_text(239, 110, fill="white", text="Accel")
    accelX = sensor(canvas, "accel", "x", 202, 150)
    accelY = sensor(canvas, "accel", "y", 232, 150)
    accelZ = sensor(canvas, "accel", "z", 262, 150)

    maglabel = canvas.create_text(392, 110, fill="white", text="Mag")
    magX = sensor(canvas, "mag", "x", 355, 150)
    magY = sensor(canvas, "mag", "y", 385, 150)
    magZ = sensor(canvas, "mag", "z", 415, 150)

    # Seperator
    seperator = ttk.Separator(root, name="sep", orient="horizontal")
    seperator.grid(column=0, row=2, columnspan=4, sticky=(N, S, E, W), pady=12)

    # Connect button
    porttext = StringVar(value="/dev/cu.usbmodem101")
    portinput = ttk.Entry(root, textvariable=porttext)
    connectbutton = ttk.Button(root, text="Connect")
    portinput.grid(column=0, row=3, columnspan=2, sticky=(N, S, E, W))
    connectbutton.grid(column=0, row=4, columnspan=2)

    # Firmware upload
    filetext = StringVar(value="/Users/dillon/projects/PicoRocket/build/rocket.uf2")
    fileinput = ttk.Entry(root, textvariable=filetext)
    uploadbutton = ttk.Button(root, text="Upload")
    fileinput.grid(column=2, row=3, columnspan=2, sticky=(N, S, E, W))
    uploadbutton.grid(column=2, row=4, columnspan=2)

    # Handler
    handle = handler(
        servobar1,
        servobar2,
        servobar3,
        servobar4,
        accelX,
        accelY,
        accelZ,
        magX,
        magY,
        magZ,
        gyroX,
        gyroY,
        gyroZ,
        portinput,
        fileinput,
        canvas,
        root,
    )

    connectbutton["command"] = handle.connect
    uploadbutton["command"] = handle.upload

    root.mainloop()


if __name__ == "__main__":
    main()
