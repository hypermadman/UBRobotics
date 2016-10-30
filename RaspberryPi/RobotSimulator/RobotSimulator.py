#Python 2.7 compatible, do not use Python 3. Make sure host computer has tkinter and PIL libraries. On debian-based OSes use 'apt-get' to acquire libraries from the internet

from Tkinter import *
from PIL import Image, ImageTk
import tkFont
import ConfigParser #to save settings
import os.path #check file exists
import time
import numpy
#import subprocess #Run bash scripts within python
from eurobot import *
from pathPrint import pathPrint
from controller import controller
import Tkconstants, tkFileDialog

global primaryRobot, datum, boundsize, firstPoint, measureLine, isRecording, angleLine, secondPoint, pixeltoCM, arenaImageScale #make global so all functions can access it
global datumColour,boundColour,boundThickness,dotsColour, dotsRadius,reversegear
global printer,boardFileName,arenaImageScale,arenarotation

################################## SET PROPERTIES HERE ####################################

datumRadius = 10			#Radius of datum marker (circle)
mainWindow = [800,600]			#Size of Main Window (The one that shows the arena)
primaryWindow = [350,250]		#Size of Primary Robot Controller Window
arenaWindow = [400,400]			#Size of Arena Controller Window
filepath = "config.ini" 		#Filename of configuration file
boardFileName = "colour.jpg" 		#Filename of board image (must be portrait orientation)
scaletoArena = True 			#Scale main window to arena image size
arenaImageScale = [1.0,1.0] 		#Scale of arena (Xscale, Yscale) image where 1.0 is original size
arenarotation = 90 			#Amount to rotate original arena picture (degrees) DEFAULT is +90 for 90 deg anti-clockwise
autoLoad = True 			#Load configuration file on window load

#Visual Properties

dotsColour = ["red","blue","green"] 	#Colours of the three measurement dots
dotsRadius = 5 				#Size of the measurement dots
datumColour = "pink" 			#Colour of datum point
boundColour = "red" 			#Colour of the arena boundary marker
boundThickness = 2 			#Thickness of arena boundary marker
pathLineColour = "red" 			#Colour of path recording line

#################################### END PROPERTIES #######################################

printer = ""
recording = False
isRecording = False #Path recording is initially off
Config = ConfigParser.ConfigParser() #init object for storing .ini config files
#import RPi.GPIO as GPIO

datum = [0,0] #Zero co-ordinate of the board (bottom right)
boundsize = [0,0] #Size of boundary
pixeltoCM = [0,0] #Used to calibrate horizontal and vertical pixels to actual distance in cm. Divide a number by this to convert to cm, multiply for pixels.
firstPoint = [0,0] #Co-ordinate of first point of measuring line
secondPoint = [0,0]

win = Tk() #Create a Tkinter window object for ARENA
win.title("Robot Simulator")
win.geometry(str(mainWindow[0])+"x"+str(mainWindow[1]))

myFont = tkFont.Font(family = 'Helvetica', size = 12, weight = 'bold')
w = Canvas(win) #canvas module to hold images, shapes
w.pack(expand=YES,fill=BOTH)

measureLine = w.create_line(0,0,0,0)
angleLine = w.create_line(0,0,0,0)


class drawcircleradius:
	def __init__(self,canv,x,y,rad,colour):
		self.thing = canv.create_oval(x-rad,y-rad,x+rad,y+rad,width=0,fill=colour)

#Initialise the measurement markers 
targetPoint = drawcircleradius(w,0,0,0,'blue')
targetPoint2 = drawcircleradius(w,0,0,0,'blue')
targetPoint3 = drawcircleradius(w,0,0,0,'blue')

#Mark datum
datumPoint = drawcircleradius(w,0,0,datumRadius,'pink')
#Mark boundary walls
boundary = w.create_rectangle(0,0,0,0,outline='red',width=3)

#Controller Window
win2 = Toplevel() #Create a Tkinter window object for controller TOPLEVEL when you have more than one
win2.title("Global Settings")
win2.geometry(str(arenaWindow[0])+"x"+str(arenaWindow[1]))
w2 = Canvas(win2,bg="white")
w2.pack(expand=YES,fill=BOTH) #let it fill the entire window

#Datum text label and input fields
t1 = Label(w2,text="Datum X: ").grid(row=0,column=0,pady=(20,0))
e1 = Entry(w2) #DatumX
e1.insert(0,datum[0]) #set input field
e1.grid(row=0,column=1,pady=(20,0)) #grid allows for easy positioning of canvas elements
t2 = Label(w2,text="Datum Y: ").grid(row=1,column=0)
e2 = Entry(w2) #DatumY
e2.insert(0,datum[1])
e2.grid(row=1,column=1)
#Arena size
t3 = Label(w2,text="Arena length: ").grid(row=3,column=0)
e3 = Entry(w2) #DatumX
e3.insert(0,0)
e3.grid(row=3,column=1)
t4 = Label(w2,text="Arena width: ").grid(row=4,column=0)
e4 = Entry(w2) #DatumY
e4.insert(0,0)
e4.grid(row=4,column=1)
#Label(w2,bg="white",text="LMB to place position marker, RMB to place distance marker.").grid(row=6,column=0,columnspan=2)
#Label(w2,bg="white",text="Place a third point with MMB after placing the two markers to measure an angle.").grid(row=7,column=0,columnspan=2)
suppressOutput = IntVar()
Checkbutton(w2,text="Suppress measurement outputs", variable=suppressOutput).grid(row=10,column=0,pady=(20,0))       

def setArena():
    global datumColour,boundColour,boundThickness,datum, boundsize, pixeltoCM,photo,boardFileName
    datum = [eval(e1.get()),eval(e2.get())] #get datum and boundary size from text input fields above
    boundsize = [eval(e3.get()),eval(e4.get())] #Evaluate string to numbers
    pixeltoCM[0] = float(e3.get()) / 300 #Horizontal length is 300 cm, so work out the ratio of pixel to cm using the arena width and length taken from boundsize
    pixeltoCM[1] = float(e4.get()) / 200 #Vertical length is 200 cm
    print("Settings updated.")
    #now redraw datum and boundary
    w.delete("all") #delete all images from previous setting
    
    image = Image.open(boardFileName) #import board image
    width, height = image.size #Pass the size to variables
    image = image.resize((int(width*arenaImageScale[0]), int(height*arenaImageScale[1])), Image.ANTIALIAS)
    photo = ImageTk.PhotoImage(image.rotate(arenarotation)) #convert for ImageTK format 90deg
    height, width = image.size #inverted order of height width as the image is rotated 90deg
    board = w.create_image(width/2,height/2,image = photo); #add image object to main canvas 'w'

    if scaletoArena: #If the window is set to scale to the size of the arena
            win.geometry(str(width)+"x"+str(height))

    height, width = image.size
    board = w.create_image(width/2,height/2,image = photo); #add image object to main canvas 'w'
    datumPoint = drawcircleradius(w,datum[0],datum[1],datumRadius,datumColour) #draw datum point as circle
    boundary = w.create_rectangle(datum[0]-boundsize[0],datum[1]-boundsize[1],datum[0],datum[1],outline=boundColour,width=boundThickness) #draw arena boundaries

def printcoords(event):
    global targetPoint,pixeltoCM,datum,firstPoint,targetPoint2,measureLine,targetPoint3, reddot, dotsColour, dotsRadius

    if isRecording == False:
        reddot = False
        firstPoint = [event.x,event.y]
        w.delete(targetPoint.thing)
        w.delete(targetPoint2.thing)
        w.delete(targetPoint3.thing)
        w.delete(measureLine)
        w.delete(angleLine)
        targetPoint = drawcircleradius(w,event.x,event.y,dotsRadius,dotsColour[0])
	if suppressOutput.get() == 0:
        	print("Clicked co-ordinate (X,Y):")
        	print(round(-(event.x - datum[0])/pixeltoCM[0],1),round(-(event.y - datum[1])/pixeltoCM[1],1))
        	print

def linecoords(event):
    global dotsRadius,firstPoint,pixeltoCM,datum,targetPoint2,dotsColour,measureLine,isRecording,targetPoint3,secondPoint,reddot,linevector
    
    if isRecording == False:
        reddot = True
        secondPoint = [event.x,event.y]
        w.delete(targetPoint2.thing)
        w.delete(targetPoint3.thing)
        w.delete(measureLine)
        w.delete(angleLine)
        measureLine = w.create_line(event.x,event.y,firstPoint[0],firstPoint[1])
        linevector = [secondPoint[0]-firstPoint[0],secondPoint[1]-firstPoint[1]]
        distance = [round(-(firstPoint[0] - event.x)/pixeltoCM[0],1),round((firstPoint[1] - event.y)/pixeltoCM[1],1)]
        magnitude =  numpy.array([round(-(firstPoint[0] - event.x)/pixeltoCM[0],0),round(-(firstPoint[1] - event.y)/pixeltoCM[1],0)])
        angle = round(math.degrees(math.atan2(distance[0],(distance[1] + 0.0001))),0)
	if suppressOutput.get() == 0:
		print "Displacement [dX dY]: " + "[" + str(distance[0]) + " " + str(distance[1]) + "]" 
        	print "Heading: " + str(-angle) + "deg from N"
                print "Distance:" + str(round(numpy.linalg.norm(magnitude),2))
        	print;
        targetPoint2 = drawcircleradius(w,event.x,event.y,dotsRadius,dotsColour[1])

def anglemeasure(event):
    global targetPoint3, dotsRadius,angleLine, secondPoint, isRecording, reddot, linevector, dotsColour

    if isRecording == False and reddot == True:
        w.delete(targetPoint3.thing)
        w.delete(angleLine)
        targetPoint3 = drawcircleradius(w,event.x,event.y,dotsRadius,dotsColour[2])
        angleLine = w.create_line(event.x,event.y,secondPoint[0],secondPoint[1])
        secondvector = [event.x - secondPoint[0],event.y - secondPoint[1]]

        #Dot product of A&B = |A||B|cos(theta)
        dot = numpy.vdot(linevector,secondvector)
        costheta = dot/numpy.linalg.norm(linevector)/numpy.linalg.norm(secondvector)
        theta = math.acos(costheta)
	if suppressOutput.get() == 0:
		print "Angle of Triangle:"
        	print str(180-int(math.degrees(theta))) + "deg"
        	print        

def saveSettings(): #Save all settings here
    cfgfile = open(filepath,'w')
    global boardFileName,primaryController,secondaryController,arenaImageScale,arenarotation
    try:        
        Config.add_section('Arena')
        Config.add_section('Primary')
        Config.add_section('Secondary')
    except:
        print "Config file already exists."
    
    result = tkMessageBox.askquestion("Save configuration?","Are you sure you want to save? Previous config file will be overwritten.", icon='warning')
    if result == 'yes':
        Config.set('Arena','datumx',e1.get())
        Config.set('Arena','datumy',e2.get())
        Config.set('Arena','length',e3.get())
        Config.set('Arena','width',e4.get())
        Config.set('Arena','boardfilename',boardFileName)
        Config.set('Arena','scale',arenaImageScale[0])
        Config.set('Arena','rotation',arenarotation)
        Config.set('Primary','startx',primaryController.e5.get())
        Config.set('Primary','starty',primaryController.e6.get())
        Config.set('Primary','heading',primaryController.e7.get())
        Config.set('Primary','track',primaryController.e8.get())
        Config.set('Primary','diameter',primaryController.e9.get())
        Config.set('Secondary','startx',secondaryController.e5.get())
        Config.set('Secondary','starty',secondaryController.e6.get())
        Config.set('Secondary','heading',secondaryController.e7.get())
        Config.set('Secondary','track',secondaryController.e8.get())
        Config.set('Secondary','diameter',secondaryController.e9.get())

        Config.write(cfgfile)
        cfgfile.close()
        print "Config saved"
    else:
        print "Save failed."

def loadSettings(): #Load all settings here
    global primaryController,secondaryController,boardFileName,arenaImageScale,arenarotation
    try:
        try:
            primaryController.closeWin()
            secondaryController.closeWin()
        except:
            print "No robot windows open"

        Config.read(filepath)
        #result = tkMessageBox.askquestion("Load configuration?","Are you sure you want to load? Any modifications will be deleted.", icon='warning')
        #if result == 'yes':
        e1.delete(0, END)
        e1.insert(0, Config.getint('Arena','datumx'))
        e2.delete(0, END)
        e2.insert(0, Config.getint('Arena','datumy'))
        e3.delete(0, END)
        e3.insert(0, Config.getint('Arena','length'))
        e4.delete(0, END)
        e4.insert(0, Config.getint('Arena','width'))
        boardFileName = Config.get('Arena','boardfilename')
        arenaImageScale[0] = Config.getfloat('Arena','scale')
        arenaImageScale[1] = Config.getfloat('Arena','scale')
        arenarotation = Config.getint('Arena','rotation')
        setArena()
        primaryController = controller(w,primaryWindow,pixeltoCM,datum,boundsize,"Primary","white","blue")
        primaryController.e5.delete(0, END)
        primaryController.e5.insert(0, Config.getint('Primary','startx'))
        primaryController.e6.delete(0, END)
        primaryController.e6.insert(0, Config.getint('Primary','starty'))
        primaryController.e7.delete(0, END)
        primaryController.e7.insert(0, Config.getint('Primary','heading'))
        primaryController.e8.delete(0, END)
        primaryController.e8.insert(0, Config.getint('Primary','track'))
        primaryController.e9.delete(0, END)
        primaryController.e9.insert(0, Config.getint('Primary','diameter'))
        primaryController.setPrimary()
        secondaryController = controller(w,primaryWindow,pixeltoCM,datum,boundsize,"Secondary","black","yellow")
        secondaryController.e5.delete(0, END)
        secondaryController.e5.insert(0, Config.getint('Secondary','startx'))
        secondaryController.e6.delete(0, END)
        secondaryController.e6.insert(0, Config.getint('Secondary','starty'))
        secondaryController.e7.delete(0, END)
        secondaryController.e7.insert(0, Config.getint('Secondary','heading'))
        secondaryController.e8.delete(0, END)
        secondaryController.e8.insert(0, Config.getint('Secondary','track'))
        secondaryController.e9.delete(0, END)
        secondaryController.e9.insert(0, Config.getint('Secondary','diameter'))
        secondaryController.setPrimary()
        print "Config loaded"
        #else:
        #    print "Load failed."
    except:
        tkMessageBox.showerror("Error!","Config file doesn't exist. Please save before loading.")
        primaryController = controller(w,primaryWindow,pixeltoCM,datum,boundsize)
        secondaryController = controller(w,primaryWindow,pixeltoCM,datum,boundsize)

#Button(w2, text="Update arena", command=setArena).grid(row=8,column=0)
#Button(w2, text="Save configuration", command=saveSettings).grid(row=9,column=0)
#Button(w2, text="Load configuration", command=loadSettings).grid(row=9,column=1)

#Change datum point to red marker
def markdatum():
    global firstPoint,boundsize,datum
    e1.delete(0,END)
    e2.delete(0,END)
    e3.delete(0,END)
    e4.delete(0,END)
    e1.insert(0,firstPoint[0]) #set input field
    e2.insert(0,firstPoint[1])
    e3.insert(0,boundsize[0]+(firstPoint[0]-datum[0]))
    e4.insert(0,boundsize[1]+(firstPoint[1]-datum[1]))
    setArena()

def markbounds():
    global firstPoint, datum,boundsize
    e3.delete(0,END)
    e4.delete(0,END)
    e3.insert(0,-firstPoint[0]+datum[0]) #set input field
    e4.insert(0,-firstPoint[1]+datum[1])
    setArena()

def squareness(): #Check if the image is not skewed. Skewed aspect ratio of images ruin movement
    global boundsize

    aspect = float(boundsize[0]) / float(boundsize[1]) #Aspect ratio of image, which should be same as 3m x 2m
    squareness = aspect/1.5
    #print aspect
    #print squareness

    tkMessageBox.showinfo("Arena image squareness","Squareness of image:"+"\n"+str(round(squareness,3))+"\n\nMust be as close as possible to 1. Badly skewed images must be correctly resized to the correct aspect ratio.\n\nOversquared images (>1) are too long in length, undersquared images are too short in length. Length is the 3m side of arena.\n\nOnly use this function once arena datum and dimensions are correctly set.")

def loadimage():
    global boardFileName
    newboardFileName = tkFileDialog.askopenfilename(title="Select arena image",filetypes=(("JPEG files","*.jpg"),("PNG files","*.png"),("All Files","*.*")))
    if newboardFileName: #If string is not empty
        boardFileName = newboardFileName
        setArena()

def expand(magnitude):
    global arenaImageScale

    arenaImageScale[0] += magnitude
    arenaImageScale[1] += magnitude
    setArena()

def rotatearena(degrees):
    global arenarotation

    arenarotation += degrees
    setArena()

menubar = Menu(win2)
win2.config(menu=menubar)

filemenu = Menu(menubar)
filemenu.add_command(label="Load config", command=loadSettings)
filemenu.add_command(label="Save config", command=saveSettings)
filemenu.add_command(label="Open arena image..", command=loadimage)
menubar.add_cascade(label="File",menu=filemenu)
 
viewmenu = Menu(menubar)
viewmenu.add_command(label="Rotate arena 90 degrees CCW", command=lambda: rotatearena(90))
viewmenu.add_command(label="Expand arena", command=lambda: expand(0.1))
viewmenu.add_command(label="Shrink arena", command=lambda: expand(-0.1))
menubar.add_cascade(label="View",menu=viewmenu)

arenamenu = Menu(menubar)
arenamenu.add_command(label="Update", command=setArena)
arenamenu.add_command(label="Set arena datum to red dot", command=markdatum)
arenamenu.add_command(label="Set arena boundary to red dot", command=markbounds)
arenamenu.add_command(label="Check squareness of image", command=squareness)
menubar.add_cascade(label="Arena",menu=arenamenu)
 
if autoLoad: #Do you want to load config file on window load?
	loadSettings()

#################### MOUSE AND KEYBINDS #######################

w.bind("<Button 1>",printcoords, add ="+")	#Find co-ordinate
w.bind("<Button 3>",linecoords)		#Find distance
w.bind("<Button 2>",anglemeasure)	#Find angle

####################### END KEYBINDS ##########################

mainloop()
