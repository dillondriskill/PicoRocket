from tkinter import *
from tkinter import ttk


# Servo Bar Class
class servobar:
    width = 100
    height = 15
    range = 45

    def __init__(self, canv: Canvas, servonum: int, x: int, y: int) -> None:
        self.canvas = canv
        self.servonum = servonum
        self.outline = 'white'
        self.fill = 'black'
        
        self.x = x
        self.y = y
        self.fillX = (self.x + self.width/2)
        self.fillY = self.y + 3
        self.fillWidth = 0
        self.fillHeight = 9
        self.angle = 0

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
            fill='grey',
            tags=('servo' + str(servonum), 'fillBar')
        )
        self.middleline = self.canvas.create_line(
            (self.x + self.width/2),
            self.y,
            (self.x + self.width/2),
            self.y + self.height,
            fill=self.outline,
            dash='.',
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
        self.fillWidth = self.x + (((self.width/2)/(self.range-3)) * self.angle)
        self.canvas.itemconfigure(self.fillBar, width=self.fillWidth)
        self.canvas.itemconfigure(self.angleLabel, text='Angle: ' )


# Window
root = Tk()
root.title('Host Tools')
root.resizable(False, False)
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

# Canvas
canvas = Canvas(root, background='black', width=480, height=320)
canvas.grid(column=0, row=0, columnspan=4, sticky=(N, W, E, S))

# servo rectangles
servobar1 = servobar(canvas, 1, 25, 30)
servobar2 = servobar(canvas, 2, 135, 30)
servobar3 = servobar(canvas, 3, 245, 30)
servobar4 = servobar(canvas, 4, 355, 30)

# Adjustment buttons



# Main loop
while 1:
    root.update_idletasks()
    root.update()
