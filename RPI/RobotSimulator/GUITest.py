from Tkinter import *
from PIL import Image, ImageTk
import tkFont
#import RPi.GPIO as GPIO

datum = [0,0] #Zero co-ordinate of the board (bottom right)
boundsize = [0,0] #Size of boundary
datumRadius = 10
logoPadding = 20 #horizontal padding around controller logo

win = Tk() #Create a Tkinter window object for ARENA
win.title("Robot Simulator")
win.geometry('800x600')

myFont = tkFont.Font(family = 'Helvetica', size = 12, weight = 'bold')

w = Canvas(win) #canvas module to hold images, shapes
w.pack(expand=YES,fill=BOTH)

image = Image.open("board.jpg") #import board image
photo = ImageTk.PhotoImage(image.rotate(90)) #convert for ImageTK format 90deg
height, width = image.size #inverted order of height width as the image is rotated 90deg
board = w.create_image(width/2,height/2,image = photo);
print(width)
print(height)

datum = [width,height]
boundsize = [width,height]

def drawcircle(canv,x,y,rad):
    canv.create_oval(x-rad,y-rad,x+rad,y+rad,width=0,fill='blue')

#Mark datum
datumPoint = drawcircle(w,width,height,datumRadius)
#mark boundary walls
boundary = w.create_rectangle(0,0,width,height,outline='red',width=3)

def exitProgram():
    print("Exit")
    win.quit()

exitButton = Button(win, text = "Exit", font = myFont, command = exitProgram, height = 2, width = 8)
#exitButton.pack(side = BOTTOM)

#Controller Window
win2 = Toplevel() #Create a Tkinter window object for controller TOPLEVEL when you have more than one
win2.title("Controller")
win2.geometry('300x400')
w2 = Canvas(win2,bg="white")
w2.pack(expand=YES,fill=BOTH)

#image = Image.open("logo.jpg") #import logo image
#width,height = image.size
#aspect = width/height
#logophoto = ImageTk.PhotoImage(image = image.resize((300-logoPadding*2,(300-logoPadding*2)/aspect))) #convert for ImageTK format, resize proportionally
#w2.create_image(150,50,image=logophoto)

#Datum
t1 = Label(w2,text="Datum X: ").grid(row=0,column=0)
e1 = Entry(w2) #DatumX
e1.insert(0,datum[0])
e1.grid(row=0,column=1)
t2 = Label(w2,text="Datum Y: ").grid(row=1,column=0)
e2 = Entry(w2) #DatumY
e2.insert(0,datum[1])
e2.grid(row=1,column=1)
#Arena size
t3 = Label(w2,text="Arena length: ").grid(row=3,column=0)
e3 = Entry(w2) #DatumX
e3.insert(0,width)
e3.grid(row=3,column=1)
t4 = Label(w2,text="Arena width: ").grid(row=4,column=0)
e4 = Entry(w2) #DatumY
e4.insert(0,height)
e4.grid(row=4,column=1)
    
def setArena():
    datum = [int(e1.get()),int(e2.get())]
    boundsize = [int(e3.get()),int(e4.get())]
    print("Settings updated.")
    print datum
    print boundsize
    #now redraw datum and boundary
    w.delete("all") #delete all images from previous setting
    height, width = image.size
    board = w.create_image(width/2,height/2,image = photo);
    datumPoint = drawcircle(w,datum[0],datum[1],datumRadius)
    boundary = w.create_rectangle(datum[0]-boundsize[0],datum[1]-boundsize[1],datum[0],datum[1],outline='red',width=3)
    
setter = Button(w2, text="Set new datum and arena size", command=setArena)
setter.grid(row=5,column=0)

setArena()

mainloop()
